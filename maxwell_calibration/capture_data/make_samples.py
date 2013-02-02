#!/usr/bin/env python

# capture samples!!!!1!one

# this script should eventually be replaced by something that finds
# samples automatically

import roslib; roslib.load_manifest("maxwell_defs"); 

import rospy
from sensor_msgs.msg import JointState

import string

header1 = """camera_measurements:
- {cam_id: head_camera, config: small_cb_4x5}
joint_commands:
- controller: head_traj_controller
  segments:
  - duration: 2.0
    positions: """

header2 = """- controller: arm_controller
  segments:
  - duration: 2.0
    positions: """

header3 = """joint_measurements:
- {chain_id: head_chain, config: tight_tol}
- {chain_id: arm_chain, config: tight_tol}
sample_id: arm_"""

header4 = """target: {chain_id: arm_chain, target_id: small_cb_4x5}"""

class SampleMaker:
    
    def __init__(self):
        rospy.init_node("make_samples")
        rospy.Subscriber("joint_states", JointState, self.callback)

        self.arm_joints = ["arm_lift_joint", "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_flex_joint", "arm_wrist_flex_joint", "arm_wrist_roll_joint"]
        self.arm_state = [0.0 for joint in self.arm_joints]
        self.head_joints = ["head_pan_joint", "head_tilt_joint"]
        self.head_state = [0.0 for joint in self.head_joints]
        self.count = 0
        
        while not rospy.is_shutdown():
            print "Move arm/head to a new sample position."
            resp = raw_input("press <enter> ")
            if string.upper(resp) == "EXIT":
                break
            else:
                # save a sample:
                count = str(self.count).zfill(4)
                f = open("samples/arm/arm_"+count+".yaml", "w")
                f.write(header1)
                print>>f, self.head_state
                f.write(header2)
                print>>f, self.arm_state
                f.write(header3)
                print>>f, count
                f.write(header4)
            self.count += 1

    def callback(self, msg):
        for i in range(len(self.arm_joints)):
            try:
                idx = msg.name.index(self.arm_joints[i])
                self.arm_state[i] = msg.position[idx]
            except: 
                pass
        for i in range(len(self.head_joints)):
            try:
                idx = msg.name.index(self.head_joints[i])
                self.head_state[i] = msg.position[idx]
            except: 
                pass


if __name__=="__main__":
    SampleMaker()

