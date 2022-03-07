#!/usr/bin/env python3

# Bezier Exploration
# Author: Lorenzo Gentilini
# E-Mail: lorenzo.gentilini6@unibo.it
# Date: February 2022
# File: gain_regressor.py 

import rospy as rp
import numpy as np
import math

from scipy.optimize import minimize
from rtree import index
from collections import namedtuple

from nav_msgs.msg import Odometry
from bezier_exploration.msg import TreeNode, gpMsgs
from geometry_msgs.msg import Point

HyperParam = namedtuple('HyperParam', 'tau_x tau_y tau_z sigma_n')

class GainRegressor:
    def __init__(self):
        # Subscribers
        self.poseSub = rp.Subscriber('/mavros/local_position/odom', Odometry, self.positionCallback)
        self.nodeSub = rp.Subscriber('/node_tree_gain', TreeNode, self.gainCallback)
        self.querySub = rp.Subscriber('/query_node', Point, self.queryCallback)

        # Publishers
        self.evalGain = rp.Publisher('/gain_eval', TreeNode, queue_size=1)
        self.gpParams = rp.Publisher('/gp_params', gpMsgs, queue_size=1)

        # Parameters
        param_tau_x = rp.get_param('gp/l_x', 2)
        param_tau_y = rp.get_param('gp/l_y', 2)
        param_tau_z = rp.get_param('gp/l_z', 2)
        param_sigma_n = rp.get_param('gp/sigma_n', 1e-3)
        self.insertionRange = rp.get_param('gain/insertion_range', 1.0)
        self.reevaluationRange = rp.get_param('gain/gp_evaluation_range', 5.0)

        self.hyperParam = HyperParam(tau_x = param_tau_x, tau_y = param_tau_y, tau_z = param_tau_z, sigma_n = param_sigma_n)
        
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.max_coord = [0, 0, 0]
        self.min_coord = [0, 0, 0]
        self.minimizigLogLikelihood = False

        # Create R-Tree
        p = index.Property()
        p.dimension = 3
        self.idx = index.Index(properties = p)
        self.id = 0
        self.idNeg = -1

        # Timers
        rp.Timer(rp.Duration(10.0), self.reevaluateGainCallback)
        rp.Timer(rp.Duration(40.0), self.reevaluateHyperParameters)

    # Helper Functions ########
    # Kernel Computation
    def gp_kernel(self, tau_x, tau_y, tau_z, xp, xq):
        np_ = xp.shape[0]
        nq_ = xq.shape[0]
        kernel = np.empty([np_, nq_])

        for i in range(0, nq_):
            for j in range(0, np_):
                dx = (xp[j, 0] - xq[i, 0])**2
                dy = (xp[j, 1] - xq[i, 1])**2
                dz = (xp[j, 2] - xq[i, 2])**2
                kernel[j, i] = np.exp(-dx/(2*tau_x**2))*np.exp(-dy/(2*tau_y**2))*np.exp(-dz/(2*tau_z**2))
        
        return kernel
    
    # Likelihood Computation
    def gp_likelihood(self, x, xd, yd):
        yd = yd.reshape(-1, 1)

        kernel = self.gp_kernel(x[0], x[1], x[2], xd, xd)
        L = np.linalg.cholesky(kernel + self.hyperParam.sigma_n**2*np.identity(kernel.shape[0]))
        alpha = np.linalg.solve(np.transpose(L), np.linalg.solve(L, yd))
        l1 = np.matmul(np.transpose(yd), alpha)

        det = np.linalg.det(kernel +  self.hyperParam.sigma_n**2*np.identity(kernel.shape[0]))

        if det < 1e-200:
            det = 1e-200

        l2 = math.log(abs(det))

        likelihood = l1 + l2
        return likelihood

    # GP Training
    def gp_train(self, y, x):
        if(y.shape[0] == 0 or x.shape[0] == 0):
            return(np.empty((0)), np.empty((0)))

        kernel = self.gp_kernel(self.hyperParam.tau_x, self.hyperParam.tau_y, self.hyperParam.tau_z, x, x)

        # Compute GP Inference
        try:
            L = np.linalg.cholesky(kernel + self.hyperParam.sigma_n**2*np.identity(kernel.shape[0]))
            alpha = np.linalg.solve(np.transpose(L), np.linalg.solve(L, y))
        except e:
            rp.logwarn("Alpha Computation Failed!")

        return alpha

    # Callback Functions ########
    # Handle Query
    def queryCallback(self, msg):
        # Add Train Point if Isolated
        boundBox = (msg.x - self.insertionRange, msg.y - self.insertionRange, msg.z - self.insertionRange,
                    msg.x + self.insertionRange, msg.y + self.insertionRange, msg.z + self.insertionRange)
        hits = self.idx.intersection(boundBox)

        if(len(list(hits)) == 0):
            node = TreeNode()
            node.x = msg.x
            node.y = msg.y
            node.z = msg.z
            node.id = self.idNeg
            self.evalGain.publish(node)

            self.idx.insert(self.idNeg, (node.x, node.y, node.z), obj = [node.x, node.y, node.z, 0, 0])
            self.idNeg -= 1

    # Handle Gain Response
    def gainCallback(self, msg):
        if msg.id > 0:
            self.idx.delete(msg.id, (msg.x, msg.y, msg.z))
            self.idx.insert(msg.id, (msg.x, msg.y, msg.z), obj = [msg.x, msg.y, msg.z, msg.yaw, msg.gain])
        else:
            self.idx.delete(msg.id, (msg.x, msg.y, msg.z))
            self.idx.insert(self.id, (msg.x, msg.y, msg.z), obj = [msg.x, msg.y, msg.z, msg.yaw, msg.gain])
            self.id += 1
            self.idNeg += 1

            if self.max_coord[0] < msg.x:
                self.max_coord[0] = msg.x
            elif self.min_coord[0] > msg.x:
                self.min_coord[0] = msg.x

            if self.max_coord[1] < msg.y:
                self.max_coord[1] = msg.y
            elif self.min_coord[1] > msg.y:
                self.min_coord[1] = msg.y

            if self.max_coord[2] < msg.z:
                self.max_coord[2] = msg.z
            elif self.min_coord[2] > msg.z:
                self.min_coord[2] = msg.z

        # Reevaluate GP
        boundBox = (self.min_coord[0], self.min_coord[1], self.min_coord[2],
                    self.max_coord[0], self.max_coord[1], self.max_coord[2])

        y = np.empty((0))
        x = np.empty((0, 4))
        hits = self.idx.intersection(boundBox, objects = True)

        for item in hits:
            if item.id > 0:
                y = np.append(y, [item.object[4] - 300], axis=0)
                x = np.append(x, [[item.object[0], item.object[1], item.object[2], item.object[3]]], axis=0)

        alpha = np.asarray(self.gp_train(y, x))

        if x.size == 0:
            return

        trained = gpMsgs()
        for xx in np.nditer(x[:,0]):
            trained.xData.append(xx)
        for yy in np.nditer(x[:,1]):
            trained.yData.append(yy)
        for zz in np.nditer(x[:,2]):
            trained.zData.append(zz)
        for ww in np.nditer(x[:,3]):
            trained.yawData.append(ww)
        for aa in np.nditer(alpha):
            trained.alpha.append(aa)

        trained.params = [self.hyperParam.tau_x, self.hyperParam.tau_y, self.hyperParam.tau_z]
        self.gpParams.publish(trained)

    # Save Current Agent Position
    def positionCallback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

    # Reevaluate Gain in a Range
    def reevaluateGainCallback(self, event):   
        if self.minimizigLogLikelihood == True:
            return

        rp.logwarn("Reevaluate Points")
        boundBox = (self.position_x - self.reevaluationRange, self.position_y - self.reevaluationRange, self.position_z - self.reevaluationRange,
                    self.position_x + self.reevaluationRange, self.position_y + self.reevaluationRange, self.position_z + self.reevaluationRange)
        hits = self.idx.intersection(boundBox, objects = True)

        for item in hits:
            node = TreeNode()
            node.x = item.object[0]
            node.y = item.object[1]
            node.z = item.object[2]
            node.id = item.id
            self.evalGain.publish(node)


    # Reevaluate Hyper Parameters
    def reevaluateHyperParameters(self, event):
        if self.minimizigLogLikelihood == True:
            return

        rp.logwarn("Reevaluate Hyperparameters")
        boundBox = (self.position_x - self.reevaluationRange, self.position_y - self.reevaluationRange, self.position_z - self.reevaluationRange,
                    self.position_x + self.reevaluationRange, self.position_y + self.reevaluationRange, self.position_z + self.reevaluationRange)
        hits = self.idx.intersection(boundBox, objects = True)

        yd = np.empty((0))
        xd = np.empty((0, 3))
        for item in hits:
            if item.id > 0:
                yd = np.append(yd - 300, [item.object[4]], axis=0)
                xd = np.append(xd, [[item.object[0], item.object[1], item.object[2]]], axis=0)

        if len(yd) < 10:
            return

        rp.loginfo("Optimizing LogLikelihood")
        
        self.minimizigLogLikelihood = True
        x0 = np.array([self.hyperParam.tau_x, self.hyperParam.tau_y, self.hyperParam.tau_z])
        
        try:
            opt = minimize(self.gp_likelihood, x0, args=(xd, yd), method='Nelder-Mead', bounds=((0.5, 4), (0.5, 4), (0.5, 4)))
            self.hyperParam = HyperParam(tau_x = opt.x[0], tau_y = opt.x[1], tau_z = opt.x[2], sigma_n = self.hyperParam.sigma_n)
        except e:
            rp.logwarn("Optimization Failed!")

        self.minimizigLogLikelihood = False

        rp.loginfo("Optimal Params: %f, %f, %f", self.hyperParam.tau_x, self.hyperParam.tau_y, self.hyperParam.tau_z)

# Main Function
if __name__ == '__main__':
    rp.init_node('gain_regressor', anonymous = False)
    GainRegressor = GainRegressor()
    rp.spin()