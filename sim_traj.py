#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time



def publish_joint_states(q_now):
    msg = JointState()
    msg.name = ["arm_joint_" + str(i) for i in range(1, 14)]
    msg.header.stamp = rospy.Time.now()
    msg.position = np.array(q_now)
    # msg.position = 180.0 / np.pi * np.array(q_now)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('sim_traj', anonymous=True)
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    record_data = np.load("../../data/data_joint.npy", allow_pickle=True)[8000:13000]
    rate = rospy.Rate(100) # 10hz
    #q_list = []
    #q0 = [0.0]*14
    #q1 = [0.0]*14
    #q1[:7] = [-30, 60, 0, -30, 0, -30, 30]
    #num = 90
    #for i in range(num):
    #    q_tmp = [0.0]*14
    #    for j in range(14):
    #        q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
    #    q_list.append(q_tmp)
    #for q in q_list:
    #    publish_joint_states(q)
    #    print(f"q: {q}")
    #    time.sleep(0.02)
    
    print(f"data size: {len(record_data)}")
    idx = 0
    forward_direction = True
    # 循环读取数据并发布
    while not rospy.is_shutdown(): # and idx <= 5:
        pub.publish(record_data[idx])
        rate.sleep()
        idx = idx + 1 if forward_direction else idx - 1
        if idx == len(record_data) - 1:
            forward_direction = False
        elif idx == 0:
            forward_direction = True
        
        print(f"record_data[{idx}]:\n")
    