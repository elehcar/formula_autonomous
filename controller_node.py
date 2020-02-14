#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import IntString


class DecisionNode(object):

    def __init__(self):
        self.orientation = None
        self.curve = False
        self.qr_distance = 200
        self.ob_linear = -1000
        self.line_linear = -1000
        self.ob_angular = -1000
        self.line_angular = -1000
        self.linear = -1000
        self.angular = -1000
        rospy.Subscriber("line_foll_topic", Twist, self.decision_callback, 0)
        rospy.Subscriber("change_obstacle", Twist, self.decision_callback, 1)
        rospy.Subscriber("planning_topic", IntString, self.decision_callback, 2)
        self.pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=1)

    def decision_callback(self, data, args):
        rotation = 0
        speed = Twist()
        if args == 0:
            self.line_linear = data.linear.x
            self.line_angular = data.angular.z
        elif args == 2:
            rotation = data.rotation
            self.qr_distance = data.distance
            self.curve = data.curve
        else:
            self.ob_linear = data.linear.x
            self.ob_angular = data.angular.z

        # se non ci sono ostacoli da considerare
        if (self.ob_linear == -1000) and (self.ob_angular == -1000):
            # se
            if (self.orientation == 0 and self.line_angular < 0) or (self.orientation == 1 and self.line_angular > 0):
                self.angular = self.line_angular * -1
            else:
                self.angular = self.line_angular
            self.orientation = None
            self.linear = self.line_linear
            print("{DECISION_NODE} SPEED==> Line: [" + str(self.linear) + "," + str(self.angular) + "]")
        # se invece ci sono gli ostacoli da considerare
        else:
            if self.ob_angular < 0:  # sinistra
                self.orientation = 0
            else:  # destra
                self.orientation = 1
            self.linear = self.ob_linear
            self.angular = self.ob_angular
            print("{DECISION_NODE} SPEED==> Obstacle: [" + str(self.linear) + "," + str(self.angular) + "]")

        if rotation == 1:
            if (self.curve and self.qr_distance > 30) or not self.curve:
                print("TURN OPPOSITE")
                self.linear = self.linear * -1
                self.qr_distance = 0
            else:
                print("Doing curve!")
        elif rotation == 2:
            self.linear = 0
            self.angular = self.angular * 2
        else:
            print("Doing great!")

        speed.linear.x = self.linear
        speed.angular.z = self.angular

        self.pub.publish(speed)
        print("{PUBLISHING: }  Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z))


if __name__ == "__main__":
    rospy.init_node("decision_node", anonymous=True)
    ob = DecisionNode()
    rospy.spin()
