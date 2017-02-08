#!/usr/bin/env python
# -*- coding: utf-8 -*-

from futaba_serial_servo import RS30X
import rospy
import rosbag
import time
from sensor_msgs.msg import JointState

class Master:
    def __init__(self):
        self.rs = RS30X.RS304MD()
        self.pub = rospy.Publisher("master_joint_state", JointState, queue_size=10)
        for i in range(1,6):
            self.rs.setTorque(i, True)
            self.rs.setAngleInTime(i, self.rs.readAngle(i), 0)
            rospy.sleep(0.01)
        for i in range(1,6):
            self.rs.setTorque(i, False)
            rospy.sleep(0.01)
        rospy.loginfo("servo initialized")

    def run(self):
       js = JointState()
       js.name=["joint{}".format(i) for i in range(1,6)]
       js.position = [max(-150,min(150,self.rs.readAngle(i))) for i in range(1,6)]
       self.pub.publish(js)
       rospy.sleep(0.01)

class Slave:
    def __init__(self):
        self.rs = RS30X.RS304MD()
        self.sub = rospy.Subscriber("/raspigibbon/master_joint_state", JointState, self.joint_callback, queue_size=10)
        for i in range(1,6):
            self.rs.setTorque(i, True)
            rospy.sleep(0.01)
        rospy.loginfo("servo initialized")

    def joint_callback(self, msg):
        if len(msg.position) > 0:
            for i in range(1, 6):
                self.rs.setAngle(i, msg.position[i-1])
        rospy.sleep(0.01)

    def shutdown(self):
        for i in range(1,6):
            self.rs.setTorque(i, False)
            rospy.sleep(0.01)
        rospy.loginfo("set all servo torque_off")

class Recoder:
    def __init__(self):
        self.jointstate = JointState()
        self.jointlog = rosbag.Bag('jointlog.bag', 'w')
        self.sub = rospy.Subscriber("/raspigibbon/master_joint_state", JointState, self.joint_callback, queue_size=10)

    def joint_callback(self, msg):
        self.jointstate.name = ["joint{}".format(i) for i in range(1,6)]
        self.jointstate.position = []
        if len(msg.position) > 0:
            for i in range(1,6):
                self.jointstate.position.append(msg.position[i-1])

    def run(self):
        self.jointlog.write('/raspigibbon/master_joint_state', self.jointstate)

    def close(self):
        self.jointlog.close()

class Player:
    def __init__(self):
        self.pub = rospy.Publisher("master_joint_state", JointState, queue_size=10)

    def run(self, msg):
        if not rospy.is_shutdown():
            js = JointState()
            js.name=["joint{}".format(i) for i in range(1,6)]
            if len(msg.position) > 0:
                for i in range(1,6):
                    js.position.append(msg.position[i-1])
            self.pub.publish(js)
            rospy.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node("raspigibbon_master_slave")
    print 'init node'
    master = Master()
    timer = 10.0
    js = JointState()
    recoder = Recoder()
    start = time.time()
    print 'start timer'

    try:
        while time.time() < (timer + start):
            if not rospy.is_shutdown():
                master.run()
                recoder.run()
        recoder.close()

        slave = Slave()
        rospy.on_shutdown(slave.shutdown)
        player = Player()
        for topic, msg, t in rosbag.Bag('jointlog.bag').read_messages():
            player.run(msg)

    except rospy.ROSInterruptException:
        pass

