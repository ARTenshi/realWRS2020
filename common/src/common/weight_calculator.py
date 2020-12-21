#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import sys
import math
from geometry_msgs.msg import Vector3, WrenchStamped


class WeightCalculator:
    """Subscribe and hold force sensor data"""
    __CONNECTION_TIMEOUT = 20.0

    def __init__(self):
        self.__prev_force = Vector3()
        self.__current_force = Vector3()
        self.__temporary_force = None

        # Subscribe force torque sensor data from HSRB
        self.__force_torque_sensor_topic = '/hsrb/wrist_wrench/raw'
        self.__wrist_wrench_sub = None

    def start_subscriber(self):
        self.__wrist_wrench_sub = rospy.Subscriber(
            self.__force_torque_sensor_topic, WrenchStamped, self.__force_torque_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(self.__force_torque_sensor_topic, WrenchStamped,
                                   timeout=self.__CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        self.__temporary_force = None
        while self.__temporary_force is None:
            pass

        self.__prev_force.x = self.__temporary_force.x
        self.__prev_force.y = self.__temporary_force.y
        self.__prev_force.z = self.__temporary_force.z

    def end_subscriber(self):
        self.__temporary_force = None
        while self.__temporary_force is None:
            pass
        self.__current_force.x = self.__temporary_force.x
        self.__current_force.y = self.__temporary_force.y
        self.__current_force.z = self.__temporary_force.z

        self.__wrist_wrench_sub.unregister()
        del self.__wrist_wrench_sub
        self.__wrist_wrench_sub = None

    def get_gram_weight(self):
        # Calcurate square sum of difference
        __prev_force = [self.__prev_force.x, self.__prev_force.y, self.__prev_force.z]
        __current_force = [self.__current_force.x, self.__current_force.y, self.__current_force.z]
        __square_sums = sum([math.pow(b - a, 2)
                             for (a, b) in zip(__prev_force, __current_force)])
        __force_difference = math.sqrt(__square_sums)

        # Convert newton to gram
        __weight = round(__force_difference / 9.81 * 1000, 1)

        return __weight

    def __force_torque_sensor_cb(self, data):
        self.__temporary_force = data.wrench.force

