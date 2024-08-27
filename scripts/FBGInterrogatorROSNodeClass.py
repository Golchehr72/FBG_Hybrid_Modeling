#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import time
import os
import csv

from threadingBarrierEvent import costumBarrier_StopEvent

fbgMeasurement_Counter = 0

#reachedThreatNum_StopEvent = 0
class FBGInterrogatorROSNode:

    def __init__(self, target_directory, mode, num_Of_Sensing_Nodes_On_Each_Fiber, FBG_Wavelength_CSV_FileName):
        self.folderName = mode
        self.FBGWavelengthCSVFileName = FBG_Wavelength_CSV_FileName
        self.directory = target_directory
        self.numOfSensingNodesOnEachFiber = num_Of_Sensing_Nodes_On_Each_Fiber
        rospy.init_node('fbgNode', anonymous=True)

        fbg_raw_wavelengths_sub = rospy.Subscriber('/fbg_interrogator/peak_values', Float64MultiArray, self.fbg_raw_wavelengths_sub_callback)

        # Variable Initialization
        self.ms = 0.001
        self.sec = 1.0
        # self.fbg_raw_wavelengths_measured = Float64MultiArray()
        # self.fbg_raw_wavelengths_measured=[]
 #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   
    # def costumBarrier_StopEvent(self, threadLock, targetThreadNum, stopEvent):
    #     #reachedThreatNum = 0
    #     global reachedThreatNum_StopEvent

    #     with threadLock:
    #         reachedThreatNum_StopEvent += 1
    #         #print('reachedThreatNum_StopEvent: ', reachedThreatNum_StopEvent)
    #         if reachedThreatNum_StopEvent == targetThreadNum:
    #             stopEvent.set()  # Signal that barrier is reached
    #         else:
    #             while not stopEvent.is_set():
    #                 #print('reachedThreatNum_StopEvent in while:',reachedThreatNum_StopEvent)
    #                 threadLock.release()  # Release lock before waiting
    #                 stopEvent.wait()
    #                 threadLock.acquire()  # Re-acquire lock after waiting
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    def saveFBGWavelengthDataLiveStreaming(self, created_folder, startTime, stopEvent, threadLock, targetThreadNum_StopEvent, flag):
        csv_numOfFBGWavelengthMeasurement = 'numOfFBGWavelengthMeasurement.csv'
        fbgMeasurementCounter = 0

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
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
        with open(csv_numOfFBGWavelengthMeasurement, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Number of FBG Wavelength Measurement'])
        
        #print('wave',self.fbg_raw_wavelengths_measured)
        while True:
            # We don not to set a barrier for FBG wavelength data collection
            # We collect data with their time stamps at the frequency rate of the interrogator, 100 Hz.
            # Then we can perform interpolation to find wavelength data at camera time frame
        
            currentTime = time.time()
            #print('currentTime:', currentTime)
            #print('wave',self.fbg_raw_wavelengths_measured)
            timestamp = currentTime - startTime
            with threadLock:
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
            fbgMeasurementCounter += 1

            if flag.is_set():
                #print('in flag')
                #time.sleep(2)
                print('FBG flag is set')
                costumBarrier_StopEvent(threadLock,targetThreadNum_StopEvent, stopEvent)
                print('Thread of FBG break')
                break

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)    
        with open(csv_numOfFBGWavelengthMeasurement, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([fbgMeasurementCounter])    
                 
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

        
        
    def fbg_raw_wavelengths_sub_callback(self, raw_wavelengths):
        self.fbg_raw_wavelengths_measured = raw_wavelengths.data



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
