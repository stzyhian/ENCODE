#!/usr/bin/env python3
# This file is covered by the LICENSE file in the root of this project.

import torch
import torch.nn as nn
import torch.optim as optim
import torch.backends.cudnn as cudnn
import torchvision.transforms as transforms
import imp
import yaml
import time
from PIL import Image
import collections
import copy
import cv2
import os
import numpy as np

from model import *
from parser import Parser
from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def convert_pointcloud2(points, pose):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points. 
    '''

    # Structured array.
    pointsCloud = np.zeros( (points.shape[0], 1), \
        dtype={ 
            "names": ( "x", "y", "z", "intensity"), 
            "formats": ( "f4", "f4", "f4", "f4" )} )

    points = points.astype(np.float32)

    pointsCloud["x"] = points[:, 0].reshape((-1, 1))
    pointsCloud["y"] = points[:, 1].reshape((-1, 1))
    pointsCloud["z"] = points[:, 2].reshape((-1, 1))
    pointsCloud["intensity"] = points[:, 3].reshape((-1, 1))

    header = Header()

    header.stamp = rospy.Time().now()
    header.frame_id = str(pose)

    msg = PointCloud2()
    msg.header = header

    msg.height = 1
    msg.width  = points.shape[0]

    msg.fields = [
        PointField('x',  0, PointField.FLOAT32, 1),
        PointField('y',  4, PointField.FLOAT32, 1),
        PointField('z',  8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = msg.point_step * points.shape[0]
    msg.is_dense     = int( np.isfinite(points).all() )
    msg.data         = pointsCloud.tostring()

    return msg

class User():
    def __init__(self, ARCH, DATA, datadir, modeldir):
        # parameters
        self.ARCH = ARCH
        self.DATA = DATA
        self.datadir = datadir
        self.modeldir = modeldir

        rospy.init_node('net', anonymous=True)
        self.pub_cloud2 = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)

        # get the data
        self.parser = Parser(root=self.datadir,
                             test_sequences=self.DATA["split"]["test"],
                             sensor=self.ARCH["dataset"]["sensor"],
                             max_points=self.ARCH["dataset"]["max_points"],
                             batch_size=1,
                             workers=1)

        # concatenate the encoder and the head
        with torch.no_grad():
            self.model = Encode(self.ARCH,
                                self.modeldir)

        # GPU?
        self.gpu = False
        self.model_single = self.model
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        print("Infering in device: ", self.device)
        if torch.cuda.is_available() and torch.cuda.device_count() > 0:
            cudnn.benchmark = True
            cudnn.fastest = True
            self.gpu = True
            self.model.cuda()

    def infer(self):
        # do test set
        self.test_dataset = self.parser.get_test_dataset()
        self.infer_subset(loader=self.parser.get_test_set())

        print('Finished Infering')

        return

    def infer_subset(self, loader):
        # switch to evaluate mode
        self.model.eval()

        # empty the cache to infer in high res
        if self.gpu:
            torch.cuda.empty_cache()

        with torch.no_grad():
            end = time.time()

            points = self.test_dataset.getPointCloud(0)
            output_np = np.array([0, 0, 0, 0, 0, 0])
            self.pub_cloud2.publish(convert_pointcloud2(points, output_np))

            for i, (scan0, scan1) in enumerate(loader):
                if self.gpu:
                    scan0 = scan0.cuda()
                    scan1 = scan1.cuda()

                # compute output
                output = self.model(scan0, scan1)

                # get original points
                points = self.test_dataset.getPointCloud(i + 1)
                # measure elapsed time
                if torch.cuda.is_available():
                    torch.cuda.synchronize()

                output_np = output.cpu().numpy().reshape(6,)
                self.pub_cloud2.publish(convert_pointcloud2(points, output_np))

                print("Infered seq scan ", i, " in", time.time() - end, "sec")
                end = time.time()
