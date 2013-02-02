#!/usr/bin/env python
import roslib; roslib.load_manifest('maxwell_calibration')

import sys
import unittest
import rospy
import time

from calibration_estimation.full_chain import FullChainRobotParams
from calibration_estimation.urdf_params import UrdfParams
from calibration_estimation.single_transform import SingleTransform
from sensor_msgs.msg import JointState

import yaml
from pr2_calibration_launch.srv import FkTest
from numpy import *
import numpy

class LoadData(unittest.TestCase):
    def setUp(self):
        config = yaml.load(open(rospy.get_param('config_file')))
        self.robot_params = UrdfParams(rospy.get_param('robot_description'), config)

        rospy.wait_for_service('fk', 3.0)
        self._fk_ref = rospy.ServiceProxy('fk', FkTest)

    def loadCommands(self, param_name):
        command_str = rospy.get_param(param_name)
        cmds = [ [float(y) for y in x.split()] for x in command_str.strip().split('\n')]
        return cmds

    def getExpected(self, root, tip, cmd):
        resp = self._fk_ref(root, tip, cmd)
        T = matrix(zeros((4,4)), float)
        T[0:3, 0:3] = reshape( matrix(resp.rot, float), (3,3))
        T[3,3] = 1.0;
        T[0:3,3] = reshape( matrix(resp.pos, float), (3,1))
        return T

class TestMaxwellFk(LoadData):

    def run_test(self, full_chain, root, tip, cmds):
        for cmd in cmds:
            print "On Command: %s" % cmd

            expected_T = self.getExpected(root, tip, cmd)
            chain_state = JointState(position=cmd)
            actual_T = full_chain.calc_block.fk(chain_state)

            print "Expected_T:"
            print expected_T
            print "Actual T:"
            print actual_T

            self.assertAlmostEqual(numpy.linalg.norm(expected_T-actual_T), 0.0, 2)

    def test_head_tilt_link(self):
        full_chain = FullChainRobotParams('head_chain','head_tilt_link')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('head_commands')
        self.run_test(full_chain, 'torso_link', 'head_tilt_link', cmds)

    def test_kinect_head_def(self):
        full_chain = FullChainRobotParams('head_chain','head_camera_rgb_link')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('head_commands')
        self.run_test(full_chain, 'torso_link', 'head_camera_rgb_link', cmds)

    def test_kinect_head_optical(self):
        full_chain = FullChainRobotParams('head_chain','head_camera_rgb_optical_frame')
        full_chain.update_config(self.robot_params)
        cmds = self.loadCommands('head_commands')
        self.run_test(full_chain, 'torso_link', 'head_camera_rgb_optical_frame', cmds)

if __name__ == '__main__':
    import rostest
    rospy.init_node("fk_test")
    rostest.unitrun('maxwell_calibration', 'test_maxwellFk', TestMaxwellFk)
