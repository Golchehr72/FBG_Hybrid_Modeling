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

#reachedThreatNum_StopEvent = 0
class maxonMotorROSNode:

    def __init__(self, target_directory, mode, num_Of_Motors, motor_Position_CSV_FileName, motor_ID):
        self.folderName = mode
        self.motorPositionCSVFileName = motor_Position_CSV_FileName
        self.directory = target_directory
        self.numOfMotors = num_Of_Motors
        self.motorID = motor_ID
        rospy.init_node('motorNode', anonymous=True)

        if self.motorID == 1:
            motorID_position_sub=rospy.Subscriber('motorID1/measured_position_jp', Float32, self.motorID_position_sub_callback)
            motorID_velocity_sub=rospy.Subscriber('motorID1/measured_velocity_jv', Float32, self.motorID_velocity_sub_callback)
            self.motorID_set_rel_position_pub = rospy.Publisher('motorID1/set_rel_position_jp', Float32)
            self.motorID_set_velocity_pub = rospy.Publisher('motorID1/set_velocity_jv', Float32)

        elif self.motorID == 3:
            motorID_position_sub=rospy.Subscriber('motorID3/measured_position_jp', Float32, self.motorID_position_sub_callback)
            motorID_velocity_sub=rospy.Subscriber('motorID3/measured_velocity_jv', Float32, self.motorID_velocity_sub_callback)
            self.motorID_set_rel_position_pub = rospy.Publisher('motorID3/set_rel_position_jp', Float32)
            self.motorID_set_velocity_pub = rospy.Publisher('motorID3/set_velocity_jv', Float32)


        # # Motor Subscribers
        # motorID1_position_sub=rospy.Subscriber('motorID1/measured_position_jp', Float32, self.motorID1_position_sub_callback)
        # motorID3_position_sub=rospy.Subscriber('motorID3/measured_position_jp', Float32, self.motorID3_position_sub_callback)

        # # motorID1_current_sub=rospy.Subscriber('motorID1/measured_current_jc', Int32, self.motorID1_current_sub_callback)
        # # motorID3_current_sub=rospy.Subscriber('motorID3/measured_current_jc', Int32, self.motorID3_current_sub_callback)

        # motorID1_velocity_sub=rospy.Subscriber('motorID1/measured_velocity_jv', Float32, self.motorID1_velocity_sub_callback)
        # motorID3_velocity_sub=rospy.Subscriber('motorID3/measured_velocity_jv', Float32, self.motorID3_velocity_sub_callback)

        # # Motor Publishers
        # self.motorID1_set_rel_position_pub = rospy.Publisher('motorID1/set_rel_position_jp', Float32)
        # self.motorID3_set_rel_position_pub = rospy.Publisher('motorID3/set_rel_position_jp', Float32)

        # # self.motorID1_set_abs_position_pub = rospy.Publisher('motorID1/set_abs_position_jp', Float32)
        # # self.motorID3_set_abs_position_pub = rospy.Publisher('motorID3/set_abs_position_jp', Float32)

        # self.motorID1_set_velocity_pub = rospy.Publisher('motorID1/set_velocity_jv', Float32)
        # self.motorID3_set_velocity_pub = rospy.Publisher('motorID3/set_velocity_jv', Float32)

        # # self.motorID1_stop_pub = rospy.Publisher('motorID1/stop_js', Empty)
        # # self.motorID3_stop_pub = rospy.Publisher('motorID3/stop_js', Empty)


        # Variable Initialization
            
        # self.motorID_position = 0.0
        # self.motorID_velocity = 0.0
        # self.motorID_measured_position = []


        # self.motorID1_position = 0.0
        # self.motorID3_position = 0.0

        # self.motorID1_velocity = 0.0
        # self.motorID3_velocity = 0.0

        # self.motorID1_current = 0
        # self.motorID3_current = 0

        # self.motorID1_maxcurrent = 800 #mA
        # self.motorID3_maxcurrent = 800 #mA

        self.ms = 0.001
        self.sec = 1.0

        # self.motorID1_measured_position = []
        # self.motorID3_measured_position = []

        
        # self.fbg_raw_wavelengths_measured = Float64MultiArray()
        # self.fbg_raw_wavelengths_measured=[]
    
    #Position Subscribers Function
    def motorID_position_sub_callback(self, pos):
        self.motorID_position = pos.data

    def giveMotorPosition(self):
        
        return self.motorID_position
           

    # def motorID1_position_sub_callback(self, pos):
    #     self.motorID1_position = pos.data
        
    # def motorID3_position_sub_callback(self, pos):
    #     self.motorID3_position = pos.data

    # def giveMotorPosition(self, motorID):
    #     if motorID == 1:
    #         return self.motorID1_position
    #     if motorID == 3:
    #         return self.motorID3_position

    # #Velocity Subscribers Functions
    def motorID_velocity_sub_callback(self, vel):
        self.motorID_velocity = vel.data

    def giveMotorVelocity(self):
        
        return self.motorID_velocity
           

    # def motorID1_velocity_sub_callback(self, vel):
    #     self.motorID1_velocity = vel.data

    # def motorID3_velocity_sub_callback(self, vel):
    #     self.motorID3_velocity = vel.data

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
    def set_rel_position_motorID(self, position_cmd):
        msg = Float32()
        msg.data = position_cmd
        self.motorID_set_rel_position_pub.publish(msg)

    # def set_rel_position_motorID1(self, position_cmd):
    #     msg = Float32()
    #     msg.data = position_cmd
    #     self.motorID1_set_rel_position_pub.publish(msg)
    #     #rospy.sleep(1.0) 

    # def set_rel_position_motorID3(self, position_cmd):
    #     msg = Float32()
    #     msg.data = position_cmd
    #     self.motorID3_set_rel_position_pub.publish(msg)
    #     #rospy.sleep(1.0)

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
    def set_velocity_motorID(self, velocity_cmd):
        msg = Float32()
        msg.data = velocity_cmd
        self.motorID_set_velocity_pub.publish(msg)
        #rospy.sleep(4.0)    
      
    # def set_velocity_motorID1(self, velocity_cmd):
    #     msg = Float32()
    #     msg.data = velocity_cmd
    #     self.motorID1_set_velocity_pub.publish(msg)
    #     #rospy.sleep(4.0)

    # def set_velocity_motorID3(self, velocity_cmd):
    #     msg = Float32()
    #     msg.data = velocity_cmd
    #     self.motorID3_set_velocity_pub.publish(msg)
    #     #rospy.sleep(4.0)

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
                 
    def maxonMotor_run(self,created_folder, startTime):
        csv_numOfMotorPositionMeasurement = 'numOfMotorPositionMeasurement.csv'
        global positionMeasurement_Counter

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        if not os.path.exists(self.motorPositionCSVFileName):
            # CSV file initialization
            with open(self.motorPositionCSVFileName, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                if self.numOfMotors == 2:
                    # csv_writer.writerow(['Timestamp (s)', 
                    #                     'Motor ID1 Position (mm)', 'Motor ID3 Position (mm)','Motor ID1 Velocity (mm/s)','Motor ID3 Velocity (mm/s)'])
                    
                    if self.motorID == 1:
                        csv_writer.writerow(['Timestamp (s)', 
                                            'Motor ID1 Position (mm)','Motor ID1 Velocity (mm/s)'])
                    elif self.motorID == 3:
                        csv_writer.writerow(['Timestamp (s)', 
                                            'Motor ID3 Position (mm)','Motor ID3 Velocity (mm/s)'])
                    
    

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
                    # csv_writer.writerow([timestamp, 
                    #                     self.motorID1_position,self.motorID3_position,self.motorID1_velocity,self.motorID3_velocity])
                    
                    
                    csv_writer.writerow([timestamp, 
                                        self.motorID_position,self.motorID_velocity])
                    
                
        positionMeasurement_Counter += 1

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)    
        with open(csv_numOfMotorPositionMeasurement, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([positionMeasurement_Counter])


    



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
