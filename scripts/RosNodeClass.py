#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Empty

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import time
import os
import csv



positionMeasurement_Counter = 0
fbgMeasurement_Counter = 0

class ROSNode:

    def __init__(self, target_directory, mode, num_Of_Motors, motor_Position_CSV_FileName, num_Of_Sensing_Nodes_On_Each_Fiber, FBG_Wavelength_CSV_FileName):
        self.folderName = mode
        self.directory = target_directory

        self.motorPositionCSVFileName = motor_Position_CSV_FileName
        self.numOfMotors = num_Of_Motors

        self.FBGWavelengthCSVFileName = FBG_Wavelength_CSV_FileName
        self.numOfSensingNodesOnEachFiber = num_Of_Sensing_Nodes_On_Each_Fiber

        rospy.init_node('controlNode', anonymous=True)

        # Motor Subscribers
        motorID1_position_sub=rospy.Subscriber('motorID1/measured_position_jp', Float32, self.motorID1_position_sub_callback)
        motorID3_position_sub=rospy.Subscriber('motorID3/measured_position_jp', Float32, self.motorID3_position_sub_callback)

        # motorID1_current_sub=rospy.Subscriber('motorID1/measured_current_jc', Int32, self.motorID1_current_sub_callback)
        # motorID3_current_sub=rospy.Subscriber('motorID3/measured_current_jc', Int32, self.motorID3_current_sub_callback)

        motorID1_velocity_sub=rospy.Subscriber('motorID1/measured_velocity_jv', Float32, self.motorID1_velocity_sub_callback)
        motorID3_velocity_sub=rospy.Subscriber('motorID3/measured_velocity_jv', Float32, self.motorID3_velocity_sub_callback)

        # Motor Publishers
        self.motorID1_set_rel_position_pub = rospy.Publisher('motorID1/set_rel_position_jp', Float32)
        self.motorID3_set_rel_position_pub = rospy.Publisher('motorID3/set_rel_position_jp', Float32)

        # self.motorID1_set_abs_position_pub = rospy.Publisher('motorID1/set_abs_position_jp', Float32)
        # self.motorID3_set_abs_position_pub = rospy.Publisher('motorID3/set_abs_position_jp', Float32)

        self.motorID1_set_velocity_pub = rospy.Publisher('motorID1/set_velocity_jv', Float32)
        self.motorID3_set_velocity_pub = rospy.Publisher('motorID3/set_velocity_jv', Float32)

        # self.motorID1_stop_pub = rospy.Publisher('motorID1/stop_js', Empty)
        # self.motorID3_stop_pub = rospy.Publisher('motorID3/stop_js', Empty)

        # FBG Subscriber
        fbg_raw_wavelengths_sub = rospy.Subscriber('/fbg_interrogator/peak_values', Float64MultiArray, self.fbg_raw_wavelengths_sub_callback)



        # Variable Initialization
        self.motorID1_position = 0.0
        self.motorID3_position = 0.0

        self.motorID1_velocity = 0.0
        self.motorID3_velocity = 0.0

        self.motorID1_current = 0
        self.motorID3_current = 0

        self.motorID1_maxcurrent = 800 #mA
        self.motorID3_maxcurrent = 800 #mA

        self.ms = 0.001
        self.sec = 1.0

        self.motorID1_measured_position = []
        self.motorID3_measured_position = []

        
        # self.fbg_raw_wavelengths_measured = Float64MultiArray()
        # self.fbg_raw_wavelengths_measured=[]
    
    #Position Subscribers Function
    def motorID1_position_sub_callback(self, pos):
        self.motorID1_position = pos.data
        
    def motorID3_position_sub_callback(self, pos):
        self.motorID3_position = pos.data

    # def giveMotorPosition(self, motorID):
    #     if motorID == 1:
    #         return self.motorID1_position
    #     if motorID == 3:
    #         return self.motorID3_position

    # #Velocity Subscribers Functions
    def motorID1_velocity_sub_callback(self, vel):
        self.motorID1_velocity = vel.data

    def motorID3_velocity_sub_callback(self, vel):
        self.motorID3_velocity = vel.data

    # def giveMotorVelocity(self, motorID):
    #     if motorID == 1:
    #         return self.motorID1_velocity
    #     if motorID == 3:
    #         return self.motorID3_velocity

    # #Current Subscribers Functions
    # def motorID1_current_sub_callback(self, cu):
    #     self.motorID1_current = cu.data

    # def motorID3_current_sub_callback(self, cu):
    #     self.motorID3_current = cu.data

    # #Set rel position
    def set_rel_position_motorID1(self, position_cmd):
        msg = Float32()
        msg.data = position_cmd
        self.motorID1_set_rel_position_pub.publish(msg)
        #rospy.sleep(1.0) 

    def set_rel_position_motorID3(self, position_cmd):
        msg = Float32()
        msg.data = position_cmd
        self.motorID3_set_rel_position_pub.publish(msg)
        #rospy.sleep(1.0)

    # #Set abs position
    # def set_abs_position_motorID1(self, position_cmd):
    #     msg = Float32()
    #     msg.data = position_cmd
    #     self.motorID1_set_abs_position_pub.publish(msg)
    #     rospy.sleep(7.0) 

    # def set_abs_position_motorID3(self, position_cmd):
    #     msg = Float32()
    #     msg.data = position_cmd
    #     self.motorID3_set_abs_position_pub.publish(msg)
    #     rospy.sleep(7.0)

    # #Set velocity
    def set_velocity_motorID1(self, velocity_cmd):
        msg = Float32()
        msg.data = velocity_cmd
        self.motorID1_set_velocity_pub.publish(msg)
        #rospy.sleep(4.0)

    def set_velocity_motorID3(self, velocity_cmd):
        msg = Float32()
        msg.data = velocity_cmd
        self.motorID3_set_velocity_pub.publish(msg)
        #rospy.sleep(4.0)

    # #Stop Function
    # def stop_motorID1(self):
    #     msg = Empty()
    #     self.motorID1_stop_pub.publish(msg)
    #     rospy.sleep(8.0)

    # def stop_motorID3(self):
    #     msg = Empty()
    #     self.motorID3_stop_pub.publish(msg)
    #     rospy.sleep(8.0)

    # #Get Current
    # def getcurrent_motorID1(self):
    #     return self.motorID1_current  

    # def getcurrent_motorID3(self):
    #     return self.motorID3_current  
        
    def fbg_raw_wavelengths_sub_callback(self, raw_wavelengths):
        self.fbg_raw_wavelengths_measured = raw_wavelengths.data
                 
    def maxonMotor_run(self,created_folder, startTime):
        csv_numOfMotorPositionMeasurement = 'numOfMotorPositionMeasurement.csv'
        global positionMeasurement_Counter

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        if not os.path.exists(self.motorPositionCSVFileName):
            # CSV file initialization
            with open(self.motorPositionCSVFileName, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if self.numOfMotors == 2:
                    csv_writer.writerow(['Timestamp (s)', 
                                        'Motor ID1 Position (mm)', 'Motor ID3 Position (mm)','Motor ID1 Velocity (mm/s)','Motor ID3 Velocity (mm/s)'])
                    
    

        # CSV file initialization
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        if not os.path.exists(csv_numOfMotorPositionMeasurement):
            with open(csv_numOfMotorPositionMeasurement, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Motor Position Measurement'])

        currentTime = time.time()

        timestamp = currentTime - startTime
            
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        with open(self.motorPositionCSVFileName, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if self.numOfMotors == 2:
                    csv_writer.writerow([timestamp, 
                                        self.motorID1_position,self.motorID3_position,self.motorID1_velocity,self.motorID3_velocity])
                
        positionMeasurement_Counter += 1

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)    
        with open(csv_numOfMotorPositionMeasurement, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([positionMeasurement_Counter])


    def FBG_run(self,created_folder, startTime):
        csv_numOfFBGWavelengthMeasurement = 'numOfFBGWavelengthMeasurement.csv'
        global fbgMeasurement_Counter

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        if not os.path.exists(self.FBGWavelengthCSVFileName):
            # CSV file initialization
            with open(self.FBGWavelengthCSVFileName, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if self.numOfSensingNodesOnEachFiber == 3:
                    csv_writer.writerow(['Timestamp (s)', 
                                        'Ch1_wavelength_Nk3_Proximal (nm)', 'Ch1_wavelength_Nk2_Middle (nm)', 'Ch1_wavelength_Nk1_Distal (nm)',
                                        'Ch2_wavelength_Nk3_Proximal (nm)', 'Ch2_wavelength_Nk2_middle (nm)', 'Ch2_wavelength_Nk1_distal (nm)', 
                                        'Ch3_wavelength_Nk3_Proximal (nm)', 'Ch3_wavelength_Nk2_Middle (nm)', 'Ch3_wavelength_Nk1_Distal (nm)',
                                        'Ch4_wavelength_Nk3_Proximal (nm)', 'Ch4_wavelength_Nk2_Middle (nm)', 'Ch4_wavelength_Nk1_Distal (nm)'])
                    
                elif self.numOfSensingNodesOnEachFiber == 4:
                    csv_writer.writerow(['Timestamp (s)', 
                                        'Ch1_wavelength_Nk4_Proximal (nm)', 'Ch1_wavelength_Nk3_MP (nm)', 'Ch1_wavelength_Nk2_MD (nm)', 'Ch1_wavelength_Nk1_Distal (nm)',
                                        'Ch2_wavelength_Nk4_Proximal (nm)', 'Ch2_wavelength_Nk3_MP (nm)', 'Ch2_wavelength_Nk2_MD (nm)', 'Ch2_wavelength_Nk1_distal (nm)', 
                                        'Ch3_wavelength_Nk4_Proximal (nm)', 'Ch3_wavelength_Nk3_MP (nm)', 'Ch3_wavelength_Nk2_MD (nm)', 'Ch3_wavelength_Nk1_Distal (nm)',
                                        'Ch4_wavelength_Nk4_Proximal (nm)', 'Ch4_wavelength_Nk3_MP (nm)', 'Ch4_wavelength_Nk2_MD (nm)', 'Ch4_wavelength_Nk1_Distal (nm)'])
    

        # CSV file initialization
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        if not os.path.exists(csv_numOfFBGWavelengthMeasurement):
            with open(csv_numOfFBGWavelengthMeasurement, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of FBG Wavelength Measurement'])

        currentTime = time.time()

        timestamp = currentTime - startTime
            
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        with open(self.FBGWavelengthCSVFileName, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if self.numOfSensingNodesOnEachFiber == 3:
                    csv_writer.writerow([timestamp, 
                                        self.fbg_raw_wavelengths_measured[0],self.fbg_raw_wavelengths_measured[1], self.fbg_raw_wavelengths_measured[2],
                                        self.fbg_raw_wavelengths_measured[3], self.fbg_raw_wavelengths_measured[4], self.fbg_raw_wavelengths_measured[5],
                                        self.fbg_raw_wavelengths_measured[6], self.fbg_raw_wavelengths_measured[7], self.fbg_raw_wavelengths_measured[8],
                                        self.fbg_raw_wavelengths_measured[9], self.fbg_raw_wavelengths_measured[10], self.fbg_raw_wavelengths_measured[11]])
                elif self.numOfSensingNodesOnEachFiber == 4:
                    csv_writer.writerow([timestamp, 
                                        self.fbg_raw_wavelengths_measured[0],self.fbg_raw_wavelengths_measured[1], self.fbg_raw_wavelengths_measured[2], self.fbg_raw_wavelengths_measured[3],
                                        self.fbg_raw_wavelengths_measured[4], self.fbg_raw_wavelengths_measured[5], self.fbg_raw_wavelengths_measured[6], self.fbg_raw_wavelengths_measured[7],
                                        self.fbg_raw_wavelengths_measured[8], self.fbg_raw_wavelengths_measured[9], self.fbg_raw_wavelengths_measured[10], self.fbg_raw_wavelengths_measured[11],
                                        self.fbg_raw_wavelengths_measured[12], self.fbg_raw_wavelengths_measured[13], self.fbg_raw_wavelengths_measured[14], self.fbg_raw_wavelengths_measured[15]])
        fbgMeasurement_Counter += 1

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)    
        with open(csv_numOfFBGWavelengthMeasurement, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([fbgMeasurement_Counter])  



    



# if __name__ == '__main__':
#     target_directory = "/home/golchehr/bigss/catkin_ws/src/fbg_hybrid_modeling"
#     mode = 'experiment'
#     num_Of_Sensing_Nodes_On_Each_Fiber = 3
#     FBG_Wavelength_CSV_FileName = 'FBGWavelengthData.csv'
#     try:
#         cal_node = FBGInterrogatorROSNode(target_directory = target_directory, mode = mode, num_Of_Sensing_Nodes_On_Each_Fiber = num_Of_Sensing_Nodes_On_Each_Fiber, FBG_Wavelength_CSV_FileName = FBG_Wavelength_CSV_FileName)
#         time.sleep(5)
#         cal_node.run()

#     except rospy.ROSInterruptException:
#         pass

# np.savetxt('test.csv', b, delimiter=',', fmt='%s')
