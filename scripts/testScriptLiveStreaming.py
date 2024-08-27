#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32

import cv2
import os
import math
#from FBGInterrogatorROSNodeClass import FBGInterrogatorROSNode
import csv
import threading
import time
import sys
import matplotlib.pyplot as plt

from stereoVisionClass import stereoVision
from RosNodeClass import ROSNode


import curses

#Specify the directory for saving the data
#target_directory = "/home/golchehr/bigss/StereoVision/Python"
target_directory = "/home/golchehr/bigss/catkin_ws/src/fbg_hybrid_modeling"
mode = 'experiment'

#FBG
num_Of_Sensing_Nodes_On_Each_Fiber = 3
FBG_Wavelength_CSV_FileName = 'FBGWavelengthData.csv'
FBGInterrogatorFrequency = 100

#Stereo Camera
csv_Cam1 = 'Camera1_CounterandTime.csv'
csv_Cam2 = 'Camera2_CounterandTime.csv'
stereoCameraFrequency = 15

#Motor
num_Of_Motors = 2
motor_Position_CSV_FileName = 'motorPositionData.csv'
motorID = 1 #motorID = 3 negative deflection and motorID = 1 positive deflection
goalPos_mm = 6 # The linear displacement of motor in mm
goalVelocity_mmPersec = 0.4 #The velocity of the linear displacement of the motor 0.8 done
bool_CDMBending = True
bool_sendMotorPositionCommand = True
motorNumOfCycle = 4 #The number of cycles for bending and straightening of CDM
motorCycleIndex = 0
motorDriectionSign = 1 # +1 is for bending cycle and -1 is for straightening cycle

time.sleep(10)
# Instantiate the stereoCameraControl Class
stereoCamera = stereoVision(Cam1ID=cv2.CAP_FIREWIRE+1, Cam2ID=cv2.CAP_FIREWIRE+0, setTrackbar='False', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = target_directory, mode = mode, CDM_MarkerNum = 30, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties')
time.sleep(2)
created_folder = stereoCamera.create_folder()
# Instantiate the FBG Class
# FBGRosNode = ROSNode(target_directory = target_directory, mode = mode, num_Of_Motors = num_Of_Motors, motor_Position_CSV_FileName = motor_Position_CSV_FileName, num_Of_Sensing_Nodes_On_Each_Fiber = num_Of_Sensing_Nodes_On_Each_Fiber, FBG_Wavelength_CSV_FileName = FBG_Wavelength_CSV_FileName)
# time.sleep(2) # if we run the program without few milisecond delay, program will stop with error since the peakwave can not read from interrogator
# Instantiate the Motor Class
motorFBGRosNode = ROSNode(target_directory = target_directory, mode = mode, num_Of_Motors = num_Of_Motors, motor_Position_CSV_FileName = motor_Position_CSV_FileName, num_Of_Sensing_Nodes_On_Each_Fiber = num_Of_Sensing_Nodes_On_Each_Fiber, FBG_Wavelength_CSV_FileName = FBG_Wavelength_CSV_FileName)
time.sleep(15) # if we run the program without few milisecond delay, program does not have enough time to create motor ROS nodes

stdscr = curses.initscr()
curses.curs_set(0)  # Hide cursor
stdscr.clear()
stdscr.addstr(0, 0, "Press ESC to exit.")
stdscr.refresh()

stdscr.nodelay(True)  # Set non-blocking mode


syncBarrier = 0
threadCondition = threading.Condition()
stopEvent = threading.Event()
flag = threading.Event()
threadLock = threading.Lock()

startTime = time.time()
#Creat thread for each camera
threadCam1 = threading.Thread(target=stereoCamera.saveImage_PreciseSynchronization1, args = (0,created_folder, startTime, stopEvent, threadLock, threadCondition, 2, 2, flag))
stdscr.addstr(1, 0, 'thread of camera 1 start')
threadCam2 = threading.Thread(target=stereoCamera.saveImage_PreciseSynchronization1, args = (1,created_folder, startTime, stopEvent, threadLock, threadCondition, 2, 2, flag))
stdscr.addstr(2, 0, 'thread of camera 2 start')
# threadFBG = threading.Thread(target=FBGRosNode.saveFBGWavelengthDataLiveStreaming, args = (created_folder, startTime, stopEvent, threadLock, 3, flag))
# stdscr.addstr(3, 0, 'thread of FBG start')

# Start the threads
threadCam1.start()
threadCam2.start()
#threadFBG.start()

num = 0
# try:
#     while True:
        
#         time.sleep(1 / FBGInterrogatorFrequency) #sec
# except KeyboardInterrupt:
#     print("Stopping the program...")
#     flag.set()
#     time.sleep(2)
#     print('join')
#     threadFBG.join()
#     print('Threads stop')
    

key = 0
prevTime = startTime
#print('start Time:',startTime)
#print('prev Time:',prevTime)
if motorID == 1:
    motorFBGRosNode.set_velocity_motorID1(goalVelocity_mmPersec)
elif motorID == 3:
    motorFBGRosNode.set_velocity_motorID3(goalVelocity_mmPersec)
stdscr.addstr(3, 0, 'Bending: press b, Straightening: press s')
while True:
    #time.sleep(1 / FBGInterrogatorFrequency) #sec
    #key = sys.stdin.read(1)
    currentTime = time.time()

    # timeStamp = currentTime - startTime
    # stdscr.addstr(4, 0, "FBG wavelength and camera data are recording, current time in sec is: {}".format(timeStamp))  # Print the value of a at fixed position
    # stdscr.refresh()
    key = stdscr.getch()

    if key != 27:

        # Storing FBG wavelength data at the frequency of 100 Hz
        if abs(currentTime - prevTime) > 0.01: #In sec
            motorFBGRosNode.FBG_run(created_folder, startTime)
            motorFBGRosNode.maxonMotor_run(created_folder, startTime)
            prevTime = currentTime  

        # motor_currentPosition = motorFBGRosNode.giveMotorPosition(motorID)
        # stdscr.addstr(5, 0, "motor position: {}".format(motor_currentPosition))  # Print the value of a at fixed position
        # stdscr.refresh()

        # motor_currentVelocity = motorFBGRosNode.giveMotorVelocity(motorID)
        # stdscr.addstr(6, 0, "motor velocity: {}".format(motor_currentVelocity))  # Print the value of a at fixed position
        # stdscr.refresh()
        

        if motorCycleIndex < motorNumOfCycle:
             
            if key == 98: # b
        
                if motorID == 1:
                    motorDriectionSign = 1
                    goalPos = goalPos_mm
                    motorFBGRosNode.set_rel_position_motorID1(motorDriectionSign*goalPos)
                elif motorID == 3:
                    motorDriectionSign = 1
                    goalPos = goalPos_mm
                    motorFBGRosNode.set_rel_position_motorID3(motorDriectionSign*goalPos)

            if key == 115: # s
           
                if motorID == 1:
                    motorDriectionSign = -1
                    goalPos = goalPos_mm
                    motorFBGRosNode.set_rel_position_motorID1(motorDriectionSign*goalPos)
                    motorCycleIndex = motorCycleIndex + 1
                elif motorID == 3:
                    motorDriectionSign = -1
                    goalPos = goalPos_mm
                    motorFBGRosNode.set_rel_position_motorID3(motorDriectionSign*goalPos)
                    motorCycleIndex = motorCycleIndex + 1

        elif motorCycleIndex == motorNumOfCycle:
            stdscr.addstr(6, 0 , "Motor Stopped ...")
            stdscr.refresh()
                          
        #     if bool_CDMBending == True:

        #         if bool_sendMotorPositionCommand == True: #Bending Cycle
        #             if motorID == 1:
        #                 motorDriectionSign = 1
        #                 goalPos = goalPos_mm
        #                 motorFBGRosNode.set_velocity_motorID1(motorDriectionSign*goalVelocity_mmPersec)
        #                 time.sleep(0.5)
        #                 motorFBGRosNode.set_rel_position_motorID1(motorDriectionSign*goalPos)
        #                 bool_sendMotorPositionCommand = False
        #             elif motorID == 3:
        #                 motorDriectionSign = 1
        #                 goalPos = goalPos_mm
        #                 motorFBGRosNode.set_velocity_motorID3(motorDriectionSign*goalVelocity_mmPersec)
        #                 time.sleep(0.5)
        #                 motorFBGRosNode.set_rel_position_motorID3(motorDriectionSign*goalPos)     
        #                 bool_sendMotorPositionCommand = False

        #         if abs(motorDriectionSign * goalPos - motor_currentPosition) <= 0.001: # mm
        #             # Motor reach the final goal
        #             # print('Bending cycle done')
        #             bool_CDMBending = False
        #             bool_sendMotorPositionCommand = True
                    

        #     elif bool_CDMBending == False: #Straightening Cycle
                
        #         if bool_sendMotorPositionCommand == True:
        #             if motorID == 1:
        #                 motorDriectionSign = -1
        #                 goalPos = goalPos_mm
        #                 motorFBGRosNode.set_velocity_motorID1(motorDriectionSign*goalVelocity_mmPersec)
        #                 time.sleep(0.2)
        #                 motorFBGRosNode.set_rel_position_motorID1(motorDriectionSign*goalPos)  
        #                 bool_sendMotorPositionCommand = False
        #                 goalPos = 0
        #             elif motorID == 3:
        #                 motorDriectionSign = -1
        #                 goalPos = goalPos_mm
        #                 motorFBGRosNode.set_velocity_motorID3(motorDriectionSign*goalVelocity_mmPersec)
        #                 time.sleep(0.2)
        #                 motorFBGRosNode.set_rel_position_motorID3(motorDriectionSign*goalPos)   
        #                 bool_sendMotorPositionCommand = False
        #                 goalPos = 0
                
        #         if abs(goalPos - motor_currentPosition) <= 0.001:
        #             # Motor reach the final goal
        #             # print('straightening cycle done')
        #             bool_CDMBending = True
        #             bool_sendMotorPositionCommand = True
        #             motorCycleIndex = motorCycleIndex + 1

        # elif motorCycleIndex == motorNumOfCycle:
        #     stdscr.addstr(6, 0 , "Motor Stopped ...")
        #     stdscr.refresh()
            
 

    
    if key == 27: # ASCII code for ESC 

        stdscr.addstr(7, 0 , "Stopping the program...")
        stdscr.refresh()
        curses.endwin()  # Restore terminal settings
        flag.set()

        time.sleep(8)

        # stdscr.addstr(7, 0, 'join')
        # stdscr.refresh()
        print('join')
        threadCam1.join()
        threadCam2.join()
        #threadFBG.join()

        # stdscr.addstr(8, 0, 'Threads stop')
        # stdscr.refresh()
        print('Threads Stpopped')

        break


time.sleep(3)
#curses.endwin()  # Restore terminal settings
# Signal the thread to stop
rospy.signal_shutdown("User stopped the thread")
# print("Stopping the program...")
# flag.set()
# time.sleep(2)
# print('join')
# threadFBG.join()

# print('Threads stop')



# time.sleep(2) #sec
# timeStampCam1 = []
# frameCounterCam1 = []

# timeStampCam2 = []
# frameCounterCam2 = []

# dffTimeStampCam1andCam2 = []

# # Read the CSV file and extract Timestamp values
# os.chdir(stereoCamera.directory + '/' + stereoCamera.folderName + '/' + created_folder)
# with open(csv_Cam1, 'r') as csvfile:
#     csv_reader = csv.reader(csvfile)
    
#     # Skip the header row
#     next(csv_reader)
    
#     # Iterate through rows and extract Timestamp values
#     for row in csv_reader:
#         frame_counter, timestamp = row
#         timeStampCam1.append(float(timestamp))  # Assuming Timestamp values are stored as strings
#         frameCounterCam1.append(float(frame_counter))

# with open(csv_Cam2, 'r') as csvfile:
#     csv_reader = csv.reader(csvfile)
    
#     # Skip the header row
#     next(csv_reader)
    
#     # Iterate through rows and extract Timestamp values
#     for row in csv_reader:
#         frame_counter, timestamp = row
#         timeStampCam2.append(float(timestamp))  # Assuming Timestamp values are stored as strings
#         frameCounterCam2.append(float(frame_counter))

# for i in range(len(timeStampCam1)):
#     dffTimeStampCam1andCam2.append(abs((timeStampCam1[i]-timeStampCam2[i])*1000))

# plt.plot(frameCounterCam1,dffTimeStampCam1andCam2)

# plt.xlabel('Frame counter')
# plt.ylabel('Time stamp difference between camera 1 and camera 2 (ms)')
# plt.savefig('Time_stamp_difference.png')
# plt.show()

# sys.exit(0)