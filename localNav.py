#!/usr/bin/env python

import rospy
import numpy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Range, LaserScan
import math
from sklearn.neural_network import MLPClassifier
import joblib

station1, station2, station3 = 0, 0, 0
pololu0, pololu1, pololu2, pololu3, pololu4, pololu5, pololu6, pololu7 = 0,0,0,0,0,0,0,0

def callbackPololu0(msg):
    global pololu0
    pololu0 = float(msg.data)

def callbackPololu1(msg):
    global pololu1
    pololu1 = float(msg.data)

def callbackPololu2(msg):
    global pololu2
    pololu2 = float(msg.data)

def callbackPololu3(msg):
    global pololu3
    pololu3 = float(msg.data)

def callbackPololu4(msg):
    global pololu4
    pololu4 = float(msg.data)

def callbackPololu5(msg):
    global pololu5
    pololu5 = float(msg.data)

def callbackPololu6(msg):
    global pololu6
    pololu6 = float(msg.data)

def callbackPololu7(msg):
    global pololu7
    pololu7 = float(msg.data)


def ChangeNavigation():
    while not rospy.is_shutdown(): 
        pub = rospy.Publisher('change_topic', String, queue_size= 10)
        error = pololu5 - pololu1
        print(error)
        if error in range(-50, 50):
            pub.publish('a')
            print("Nawigacja lokalna") # Test nawigacji
        else:
            pub.publish('q')
            print("Nawigacja globalna") # Test nawigacji
        rospy.sleep(0.2)

def driveForward(msg):
    leftWheel = rospy.Publisher("/control_left", Float64  ,queue_size= 1)
    rightWheel = rospy.Publisher("/control_right", Float64, queue_size= 1)
    if (msg.data[0] == 'a'):
        leftWheel.publish(-50)
        rightWheel.publish(50)

def identifyStation(msg):
    temp = []
    filename = 'rotated_stations.sav'
    scan = msg.ranges
    lst = list(scan)
    for i in range(360):
        if lst[i] == math.inf:
            lst[i] = 0
    for i in range(3):
        temp.append(lst)      
    loadModel = joblib.load(filename)
    y_pred = loadModel.predict(temp)
    print(y_pred)
    global station1
    global station2
    global station3
    if y_pred[0] == 1:
        station1 +=  1
    if y_pred[0] == 2:
        station2 += 1
    if y_pred[0] == 3:
        station3 +=  1
    final = [station1, station2, station3]
    return final.index(max(final)) + 1


if __name__ == "__main__":
    rospy.init_node('scan_docking_stations')
    
    rospy.Subscriber('/change_topic', String, driveForward)
    rospy.Subscriber('/scan', LaserScan, identifyStation)
    rospy.Subscriber('/Pololu0', Float64, callbackPololu0)
    rospy.Subscriber('/Pololu1', Float64, callbackPololu1)
    rospy.Subscriber('/Pololu2', Float64, callbackPololu2)
    rospy.Subscriber('/Pololu3', Float64, callbackPololu3)
    rospy.Subscriber('/Pololu4', Float64, callbackPololu4)
    rospy.Subscriber('/Pololu5', Float64, callbackPololu5)
    rospy.Subscriber('/Pololu6', Float64, callbackPololu6)
    rospy.Subscriber('/Pololu7', Float64, callbackPololu7)
    ChangeNavigation()
    driveForward()
    print(identifyStation())