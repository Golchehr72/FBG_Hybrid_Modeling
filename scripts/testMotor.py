#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32

import cv2
import os
import math
from FBGInterrogatorROSNodeClass import FBGInterrogatorROSNode
import csv
import threading
import time
import sys
import matplotlib.pyplot as plt

from maxonMotorROSNodeClass import maxonMotorROSNode

import curses

#Specify the directory for saving the data
#target_directory = "/home/golchehr/bigss/StereoVision/Python"
target_directory = "/home/golchehr/bigss/catkin_ws/src/fbg_hybrid_modeling"
mode = 'experiment'

num_Of_Motors = 2
motor_Position_CSV_FileName = 'motorPositionData.csv'
motorID = 1
#time.sleep(20) # if we run the program without few milisecond delay, program will stop with error since the peakwave can not read from interrogator
startTime = time.time()
motorRosNode = maxonMotorROSNode(target_directory, mode, num_Of_Motors, motor_Position_CSV_FileName, 1)
time.sleep(15) # if we run the program without few milisecond delay, program will stop with error since the peakwave can not read from interrogator


created_folder = '12'
goalPos_mm = 2
goalVelocity_mmPersec = 1
bool_CDMBending = True
bool_sendMotorPositionCommand = True
motorNumOfCycle = 2
motorCycleIndex = 0
velIndex = 1
motorDriectionSign = 1
#motorRosNode.set_velocity_motorID1(motorDriectionSign*goalVelocity_mmPersec)
indexst = 0
indexbnd = 0
prevTime = startTime
while(True):
    
    #motor_currentPosition = motorRosNode.giveMotorPosition(motorID)
    currentTime = time.time()
    if abs(currentTime - prevTime) >=0.01:
        motorRosNode.maxonMotor_run(created_folder, startTime)
        prevTime = currentTime

    motor_currentPosition = motorRosNode.giveMotorPosition()
    print(f"Motor Pos: {motor_currentPosition}", end = "\r")


    if motorCycleIndex < motorNumOfCycle:   
        if bool_CDMBending == True:

            if bool_sendMotorPositionCommand == True: #Bending Cycle
                if motorID == 1:
                    if indexbnd > 500000:
                        motorDriectionSign = 1
                        goalPos_mm = 2
                        #motorRosNode.set_velocity_motorID(motorDriectionSign*goalVelocity_mmPersec)
                        #time.sleep(0.5)
                        motorRosNode.set_rel_position_motorID(motorDriectionSign*goalPos_mm)
                        time.sleep(0.5)
                        #motorRosNode.set_velocity_motorID1(motorDriectionSign*goalVelocity_mmPersec)  
                        bool_sendMotorPositionCommand = False
                        indexbnd = 0

                    else:
                        if indexbnd == 0:
                            motorDriectionSign = 1
                            motorRosNode.set_velocity_motorID(motorDriectionSign*goalVelocity_mmPersec)
                        indexbnd = indexbnd + 1
                        goalPos_mm = 2

                elif motorID == 3:
                    motorDriectionSign = 1
                    goalPos_mm = 2
                    motorRosNode.set_rel_position_motorID(motorDriectionSign*goalPos_mm)
                    motorRosNode.set_velocity_motorID(motorDriectionSign*goalVelocity_mmPersec)
                    bool_sendMotorPositionCommand = False

            # if motorRosNode.giveMotorVelocity(motorID) > 0 and velIndex == 1:
            #     motorRosNode.set_velocity_motorID1(motorDriectionSign*goalVelocity_mmPersec)
            #     velIndex = velIndex + 1

            if abs(motorDriectionSign * goalPos_mm - motor_currentPosition)<=0.001:
                # Motor reach the final goal
                #print('Bending cycle done')
                bool_CDMBending = False
                bool_sendMotorPositionCommand = True
                

        elif bool_CDMBending == False: #Straightening Cycle
            
            if bool_sendMotorPositionCommand == True:
                if motorID == 1:
                    if indexst > 500000:
                        #print('st1')
                        motorDriectionSign = -1
                        # motorRosNode.set_velocity_motorID(motorDriectionSign*0.5)
                        # time.sleep(0.2)
                        goalPos_mm = 2
                        motorRosNode.set_rel_position_motorID(motorDriectionSign*goalPos_mm) 
                        time.sleep(0.5) 
                        bool_sendMotorPositionCommand = False
                        goalPos_mm = 0
                        indexst = 0
                    else:
                        if indexst == 0:
                            motorDriectionSign = -1
                            motorRosNode.set_velocity_motorID(motorDriectionSign*0.3)
                        indexst = indexst + 1
                        goalPos_mm = 0
                        

                elif motorID == 3:
                    motorDriectionSign = -1
                    motorRosNode.set_rel_position_motorID(motorDriectionSign*goalPos_mm)
                    motorRosNode.set_velocity_motorID(motorDriectionSign*goalVelocity_mmPersec)
                    bool_sendMotorPositionCommand = False
                    goalPos_mm = 0
            
            if abs(goalPos_mm - motor_currentPosition)<=0.001:
                # Motor reach the final goal
                #print('straightening cycle done')
                bool_CDMBending = True
                bool_sendMotorPositionCommand = True
                motorCycleIndex = motorCycleIndex + 1

    elif motorCycleIndex == motorNumOfCycle:
        break

        

        
    