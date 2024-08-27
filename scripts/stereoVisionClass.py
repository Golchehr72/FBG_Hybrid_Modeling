#!/usr/bin/env python3

import cv2
import os
from datetime import datetime
import csv
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
import imutils
import glob

from threadingBarrierEvent import costumBarrier_Synchronization, costumBarrier_StopEvent

#reachedThreatNum_Cam = 0
#reachedThreatNum_StopEvent = 0

class stereoVision:
    def __init__(self, Cam1ID=cv2.CAP_FIREWIRE+0, Cam2ID=cv2.CAP_FIREWIRE+1, setTrackbar='False', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = "/home/golchehr/bigss/StereoVision/Calibration/Python", mode='calibration', CDM_MarkerNum = 30, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties'):
        #Change directory to the target directory
        os.chdir(target_directory)
        self.trackBar = setTrackbar
        #mode : calibration, fps, liveStreaming, mask
        self.folderName = mode
        self.folderNameHSVValue = folderName_HSV_Value
        self.folderNameVideoProperties = folderName_videoProperties

        self.frame_Width = frameWidth
        self.frame_Height = frameHeight

        self.CDMMarkerNum = CDM_MarkerNum

        if setTrackbar=='True':
            # Initializing trackbar variables
            self.brightnessCam1 = 120
            self.brightnessCam2 = 120

            self.redCam1 = 514
            self.redCam2 = 514

            self.blueCam1 = 712
            self.blueCam2 = 712

            self.sharpnessCam1 = 3000
            self.sharpnessCam2 = 3000

            self.gammaCam1 = 773
            self.gammaCam2 = 773

            self.saturationCam1 = 1950
            self.saturationCam2 = 1950

            self.exposureCam1 = 309
            self.exposureCam2 = 309

            self.gainCam1 = 7
            self.gainCam2 = 7

            #Trackbar Maximum values
            self.max_brightness = 255
            self.max_red = 1000
            self.max_blue = 1000
            self.max_exposure = 863
            self.max_saturation = 3000
            self.max_gain = 683
            self.max_gamma = 1000
            self.max_sharpness = 4095

        self.directory = target_directory
        self.cam1 = cv2.VideoCapture(Cam1ID)
        self.cam2 = cv2.VideoCapture(Cam2ID)
        if self.cam1.isOpened()==False or self.cam2.isOpened()==False:
            print('Error for openning cameras')
        # Camera iso speed and resolution must be set right after video capture function
        #First iso speed adjustment to allocate iso resources for increasing camera resolution
        #We must set iso speed to 800 in order to acheive high resolution
        self.cam1.set(cv2.CAP_PROP_ISO_SPEED,IsoSpeed)
        self.cam2.set(cv2.CAP_PROP_ISO_SPEED,IsoSpeed)
        time.sleep(0.5)
        print('Cam1 iso speed',self.cam1.get(cv2.CAP_PROP_ISO_SPEED))
        print('Cam2 iso speed',self.cam1.get(cv2.CAP_PROP_ISO_SPEED))

        self.cam1.set(cv2.CAP_PROP_FRAME_WIDTH,frameWidth)
        self.cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,frameHeight)
        print('Cam1 width:',self.cam1.get(cv2.CAP_PROP_FRAME_WIDTH))
        print('Cam1 height:',self.cam1.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH,frameWidth)
        self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT,frameHeight)
        print('Cam2 width:',self.cam2.get(cv2.CAP_PROP_FRAME_WIDTH))
        print('Cam2 height:',self.cam2.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.trackBar == 'False':
            self.setCam1Properties()
            self.setCam2Properties()

            self.getPropertiesCam1()
            self.getPropertiesCam2()


    #Define a function for creating a folder at a specified directory
    def create_folder(self):

        if self.trackBar == 'False':
            try:
                #get the current date and time
                current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") #Formatting date and time to string
                
                #get the current directory
                current_directory = os.getcwd()
                if current_directory!=self.directory:
                    os.chdir(self.directory)
                    if not os.path.exists(self.folderName):
                        os.mkdir(self.folderName)

                    os.chdir(self.directory + '/' + self.folderName)
                    os.mkdir(current_time)
                    print(f"Folder '{current_time}' created successfully.")

                    os.chdir(self.directory + '/' + self.folderName + '/' + current_time)
                    os.mkdir('Camera1')
                    os.mkdir('Camera2')
                    print("Folder 'Camera 1' created successfully.")
                    print(f"Folder 'Camera 2' created successfully.")
                    return current_time
                else:
                    #Create a folder name with the current date and time
                    if not os.path.exists(self.folderName):
                        os.mkdir(self.folderName)

                    os.chdir(self.directory + '/' + self.folderName)
                    os.mkdir(current_time)
                    print(f"Folder '{current_time}' created successfully.")

                    os.chdir(self.directory + '/' + self.folderName+ '/' + current_time)
                    os.mkdir('Camera1')
                    os.mkdir('Camera2')
                    print("Folder 'Camera 1' created successfully.")
                    print(f"Folder 'Camera 2' created successfully.")
                    return current_time

            except OSError as e:
                print(f"Error for creating folder: {e}")
                return None
            
        elif self.trackBar == 'True':
            #get the current directory
            current_directory = os.getcwd()
            if current_directory!=self.directory:
                    if not os.path.exists(self.folderName):
                        os.chdir(self.directory)
                        os.mkdir(self.folderName)
            else:
                #Create a folder name with the current date and time
                if not os.path.exists(self.folderName):
                    os.mkdir(self.folderName)
                 
    # Callback functions for trackbar   
    # callback function that is called every time the trackbar is moved. This function typically takes the trackbar's current value as an argument.
    def onTrackbar_brightness_Cam1(self,value):
        self.brightnessCam1 = value
    def onTrackbar_brightness_Cam2(self,value):
        self.brightnessCam2 = value

    def onTrackbar_red_Cam1(self,value):
        self.redCam1 = value
    def onTrackbar_red_Cam2(self,value):
        self.redCam2 = value

    def onTrackbar_blue_Cam1(self,value):
        self.blueCam1 = value
    def onTrackbar_blue_Cam2(self,value):
        self.blueCam2 = value

    def onTrackbar_sharpness_Cam1(self,value):
        self.sharpnessCam1 = value
    def onTrackbar_sharpness_Cam2(self,value):
        self.sharpnessCam2 = value

    def onTrackbar_gamma_Cam1(self,value):
        self.gammaCam1 = value
    def onTrackbar_gamma_Cam2(self,value):
        self.gammaCam2 = value

    def onTrackbar_saturation_Cam1(self,value):
        self.saturationCam1 = value
    def onTrackbar_saturation_Cam2(self,value):
        self.saturationCam2 = value

    def onTrackbar_exposure_Cam1(self,value):
        self.exposureCam1 = value
    def onTrackbar_exposure_Cam2(self,value):
        self.exposureCam2 = value

    def onTrackbar_gain_Cam1(self,value):
        self.gainCam1 = value
    def onTrackbar_gain_Cam2(self,value):
        self.gainCam2 = value

    def creatTrackbarCam1(self,windowName):
        cv2.namedWindow(windowName)
        cv2.createTrackbar('Brightness', windowName, self.brightnessCam1, self.max_brightness, self.onTrackbar_brightness_Cam1)
        cv2.createTrackbar('White Balance Red', windowName, self.redCam1, self.max_red, self.onTrackbar_red_Cam1)
        cv2.createTrackbar('White Balance Blue', windowName, self.blueCam1, self.max_blue, self.onTrackbar_blue_Cam1)    
        cv2.createTrackbar('Sharpness', windowName, self.sharpnessCam1, self.max_sharpness, self.onTrackbar_sharpness_Cam1)        
        cv2.createTrackbar('Gamma', windowName, self.gammaCam1, self.max_gamma, self.onTrackbar_gamma_Cam1)        
        cv2.createTrackbar('Saturation', windowName, self.saturationCam1, self.max_saturation, self.onTrackbar_saturation_Cam1)
        cv2.createTrackbar('Exposure', windowName, self.exposureCam1, self.max_exposure, self.onTrackbar_exposure_Cam1)        
        cv2.createTrackbar('Gain', windowName, self.gainCam1, self.max_gain, self.onTrackbar_gain_Cam1)

    def creatTrackbarCam2(self,windowName):
        cv2.namedWindow(windowName)
        cv2.createTrackbar('Brightness', windowName, self.brightnessCam2, self.max_brightness, self.onTrackbar_brightness_Cam2)        
        cv2.createTrackbar('White Balance Red', windowName, self.redCam2, self.max_red, self.onTrackbar_red_Cam2)
        cv2.createTrackbar('White Balance Blue', windowName, self.blueCam2, self.max_blue, self.onTrackbar_blue_Cam2)    
        cv2.createTrackbar('Sharpness', windowName, self.sharpnessCam2, self.max_sharpness, self.onTrackbar_sharpness_Cam2)        
        cv2.createTrackbar('Gamma', windowName, self.gammaCam2, self.max_gamma, self.onTrackbar_gamma_Cam2)        
        cv2.createTrackbar('Saturation', windowName, self.saturationCam2, self.max_saturation, self.onTrackbar_saturation_Cam2)
        cv2.createTrackbar('Exposure', windowName, self.exposureCam2, self.max_exposure, self.onTrackbar_exposure_Cam2)        
        cv2.createTrackbar('Gain', windowName, self.gainCam2, self.max_gain, self.onTrackbar_gain_Cam2)

    def trackbarVisualization(self,windowName1, windowName2):
        # Call the function to create the folder
        self.creatTrackbarCam1(windowName1)
        self.creatTrackbarCam2(windowName2)
        if self.cam1.isOpened()==False or self.cam2.isOpened()==False:
            print('Error for openning cameras')
        else:
            print('Press Esc to exit')
            while self.cam1.isOpened() and self.cam2.isOpened():

                succes1, img1 = self.cam1.read() # read returns tuple, so we need to write it like success1, img. Otherwise, we will face the error: mat is not a numerical tuple
                succes2, img2 = self.cam2.read()

                k = cv2.waitKey(1)

                if k == 27: #Esc key to stop the program
                    break

                self.setTrackbarCam1()
                self.setTrackbarCam2()

                cv2.imshow(windowName1,img1)
                cv2.imshow(windowName2,img2)

        # Release and destroy all windows before termination       
        self.getPropertiesCam1()
        self.getPropertiesCam2()
        self.cam1.release()
        self.cam2.release()
        cv2.destroyAllWindows()


    def capture_image_press_key(self, windowName1, windowName2):
        # Call the function to create the folder
        created_folder = self.create_folder()
        
        csv_numOfFramesCam1 = 'numOfFramesCam1.csv'
        # Initializing frame counter of Cam1
        frameCounter_Cam1 = 0

        csv_numOfFramesCam2 = 'numOfFramesCam2.csv'
        # Initializing frame counter of Cam2
        frameCounter_Cam2 = 0

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)

        # CSV file initialization
        with open(csv_numOfFramesCam1, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Number of Frames_Cam1'])

        # CSV file initialization
        with open(csv_numOfFramesCam2, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Number of Frames_Cam2'])

        if self.cam1.isOpened()==False or self.cam2.isOpened()==False:
            print('Error for openning cameras')
        else:
            num = 0
            print('Press s to capture imag, press Esc to exit')
            while self.cam1.isOpened() and self.cam2.isOpened():

                succes1, img1 = self.cam1.read() # read returns tuple, so we need to write it like success1, img. Otherwise, we will face the error: mat is not a numerical tuple
                succes2, img2 = self.cam2.read()

                k = cv2.waitKey(1)

                if k == 27: #Esc key to stop the program
                    break
                elif k == ord('s') and self.trackBar == 'False': # wait for 's' key to save and exit
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera1/' + '/image1_' + str(num+1) + '.png', img1)
                    frameCounter_Cam1 +=1
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera2/' + '/image2_' + str(num+1) + '.png', img2)
                    frameCounter_Cam2 +=1
                    print("images saved!")
                    num += 1

                cv2.imshow(windowName1,img1)
                cv2.imshow(windowName2,img2)

        # Release and destroy all windows before termination
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)        
        with open(csv_numOfFramesCam1, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([frameCounter_Cam1])
    
        with open(csv_numOfFramesCam2, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam2]) 

        self.getPropertiesCam1()
        self.getPropertiesCam2()
        self.cam1.release()
        self.cam2.release()
        cv2.destroyAllWindows()

    def setTrackbarCam1(self):
        self.cam1.set(cv2.CAP_PROP_BRIGHTNESS,self.brightnessCam1)
        self.cam1.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V,self.redCam1)        
        self.cam1.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,self.blueCam1)        
        self.cam1.set(cv2.CAP_PROP_SHARPNESS,self.sharpnessCam1)        
        self.cam1.set(cv2.CAP_PROP_GAMMA,self.gammaCam1)        
        self.cam1.set(cv2.CAP_PROP_SATURATION,self.saturationCam1)        
        self.cam1.set(cv2.CAP_PROP_EXPOSURE,self.exposureCam1)        
        self.cam1.set(cv2.CAP_PROP_GAIN,self.gainCam1)

    def setTrackbarCam2(self):
        self.cam2.set(cv2.CAP_PROP_BRIGHTNESS,self.brightnessCam2)
        self.cam2.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V,self.redCam2)        
        self.cam2.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,self.blueCam2)        
        self.cam2.set(cv2.CAP_PROP_SHARPNESS,self.sharpnessCam2)        
        self.cam2.set(cv2.CAP_PROP_GAMMA,self.gammaCam2)        
        self.cam2.set(cv2.CAP_PROP_SATURATION,self.saturationCam2)        
        self.cam2.set(cv2.CAP_PROP_EXPOSURE,self.exposureCam2)        
        self.cam2.set(cv2.CAP_PROP_GAIN,self.gainCam2)

    def getPropertiesCam1(self):

        os.chdir(self.directory)
        if not os.path.exists(self.folderNameVideoProperties):
            os.makedirs(self.folderNameVideoProperties)
            print(f"Folder '{self.folderNameVideoProperties}' created successfully")
        else:
            print(f"Folder '{self.folderNameVideoProperties}' already exists")

        os.chdir(self.directory + '/' + self.folderNameVideoProperties )

        if self.trackBar == 'True':
            csvFileName = 'videoCapturePropertiesCam1'
            videoPropertiesCam1 = {'CAP_PROP_BRIGHTNESS': self.cam1.get(cv2.CAP_PROP_BRIGHTNESS),
                                'CAP_PROP_WHITE_BALANCE_RED_V': self.cam1.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V),
                                'CAP_PROP_WHITE_BALANCE_BLUE_U': self.cam1.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
                                'CAP_PROP_SHARPNESS': self.cam1.get(cv2.CAP_PROP_SHARPNESS),
                                'CAP_PROP_GAMMA':self.cam1.get(cv2.CAP_PROP_GAMMA),
                                'CAP_PROP_SATURATION':self.cam1.get(cv2.CAP_PROP_SATURATION),
                                'CAP_PROP_EXPOSURE':self.cam1.get(cv2.CAP_PROP_EXPOSURE),
                                'CAP_PROP_GAIN':self.cam1.get(cv2.CAP_PROP_GAIN)}
            
            os.chdir(self.directory + '/' + self.folderNameVideoProperties )
            # Save camera properties in Csv file
            with open(csvFileName, 'w', newline='') as csvfile:
                fieldnames = ['property', 'value']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                for prop, value in videoPropertiesCam1.items():
                    writer.writerow({'property': prop, 'value': int(value)})

        print("Camera 1 ISO speed:", self.cam1.get(cv2.CAP_PROP_ISO_SPEED))        
        print("Camera 1 Brightness:", self.cam1.get(cv2.CAP_PROP_BRIGHTNESS))        
        print("Camera 1 Red Balance:", self.cam1.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V))        
        print("Camera 1 Blue Balance:", self.cam1.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U))        
        print("Camera 1 Sharpness:", self.cam1.get(cv2.CAP_PROP_SHARPNESS))
        print("Camera 1 Gamma:", self.cam1.get(cv2.CAP_PROP_GAMMA))        
        print("Camera 1 Saturation:", self.cam1.get(cv2.CAP_PROP_SATURATION))        
        print("Camera 1 Exposure:", self.cam1.get(cv2.CAP_PROP_EXPOSURE))        
        print("Camera 1 Gain:", self.cam1.get(cv2.CAP_PROP_GAIN))
        

    def getPropertiesCam2(self):

        os.chdir(self.directory)
        if not os.path.exists(self.folderNameVideoProperties):
            os.makedirs(self.folderNameVideoProperties)
            print(f"Folder '{self.folderNameVideoProperties}' created successfully")
        else:
            print(f"Folder '{self.folderNameVideoProperties}' already exists")

        os.chdir(self.directory + '/' + self.folderNameVideoProperties )

        if self.trackBar == 'True':
            csvFileName = 'videoCapturePropertiesCam2'
            videoPropertiesCam2 = {'CAP_PROP_BRIGHTNESS': self.cam2.get(cv2.CAP_PROP_BRIGHTNESS),
                                'CAP_PROP_WHITE_BALANCE_RED_V': self.cam2.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V),
                                'CAP_PROP_WHITE_BALANCE_BLUE_U': self.cam2.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
                                'CAP_PROP_SHARPNESS': self.cam2.get(cv2.CAP_PROP_SHARPNESS),
                                'CAP_PROP_GAMMA':self.cam2.get(cv2.CAP_PROP_GAMMA),
                                'CAP_PROP_SATURATION':self.cam2.get(cv2.CAP_PROP_SATURATION),
                                'CAP_PROP_EXPOSURE':self.cam2.get(cv2.CAP_PROP_EXPOSURE),
                                'CAP_PROP_GAIN':self.cam2.get(cv2.CAP_PROP_GAIN)}
            
            os.chdir(self.directory + '/' + self.folderNameVideoProperties )
            # Save camera properties in Csv file
            with open(csvFileName, 'w', newline='') as csvfile:
                fieldnames = ['property', 'value']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                for prop, value in videoPropertiesCam2.items():
                    writer.writerow({'property': prop, 'value': int(value)})

        print("Camera 2 ISO speed:", self.cam2.get(cv2.CAP_PROP_ISO_SPEED))
        print("Camera 2 Brightness:", self.cam2.get(cv2.CAP_PROP_BRIGHTNESS))
        print("Camera 2 Red Balance:", self.cam2.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V))
        print("Camera 2 Blue Balance:", self.cam2.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U))
        print("Camera 2 Sharpness:", self.cam2.get(cv2.CAP_PROP_SHARPNESS))
        print("Camera 2 Gamma:", self.cam2.get(cv2.CAP_PROP_GAMMA))
        print("Camera 2 Saturation:", self.cam2.get(cv2.CAP_PROP_SATURATION))
        print("Camera 2 Exposure:", self.cam2.get(cv2.CAP_PROP_EXPOSURE))
        print("Camera 2 Gain:", self.cam2.get(cv2.CAP_PROP_GAIN))

    def loadCamProperties(self, FolderName, csvFileName):
        os.chdir(self.directory + '/' + FolderName )
        properties = {}
        with open(csvFileName, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                properties[row['property']] = int(row['value'])
        return properties
    
    def setCam1Properties(self):
        propertiesCam1 = self.loadCamProperties(self.folderNameVideoProperties, 'videoCapturePropertiesCam1')
        for prop, value in propertiesCam1.items():
            # Use getattr to access the attribute by name
            prop_number = getattr(cv2, prop, None)
            self.cam1.set(prop_number,value)

    def setCam2Properties(self):
        propertiesCam2 = self.loadCamProperties(self.folderNameVideoProperties, 'videoCapturePropertiesCam2')
        for prop, value in propertiesCam2.items():
            # Use getattr to access the attribute by name
            prop_number = getattr(cv2, prop, None)
            self.cam2.set(prop_number,value)

    def saveImageCounterAndTimeToCsv_Cam1(self, csvFileName, created_folder, stopThreadCam1):
        global frameCounter_Cam1 
        frameCounter_Cam1 = 0
        
        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)

        previousTime = time.time()
        
        while True:
            
            success, frame = self.cam1.read()
            currentTime = time.time()
            if not success:
                print("Error reading frame from camera 1")
                break
            
            timestamp = currentTime - previousTime
            # Save image
            cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera1/' + '/image1_' + str(frameCounter_Cam1+1) + '.png', frame)
            # Append data to CSV file
            with open(csvFileName, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam1, timestamp])

            frameCounter_Cam1 +=1

            if stopThreadCam1.is_set():
                print('break1')
                break

            k = cv2.waitKey(1)

            if k == 27: #Esc key to stop the program
                break

        self.cam1.release()

    def saveImageCounterAndTimeToCsv_Cam2(self, csvFileName, created_folder, stopThreadCam2):
        global frameCounter_Cam2
        frameCounter_Cam2 = 0

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)

        previousTime = time.time() #second
    
        while True:    
            success, frame = self.cam2.read()
            currentTime = time.time()
            if not success:
                print("Error reading frame from camera 1")
                break
            
            timestamp = currentTime - previousTime
            # Save image
            cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera2/' + '/image2_' + str(frameCounter_Cam2+1) + '.png', frame)
            # Append data to CSV file
            with open(csvFileName, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam2, timestamp])
            
            frameCounter_Cam2 +=1

            if stopThreadCam2.is_set():
                print('break2')
                break

            k = cv2.waitKey(1)

            if k == 27: #Esc key to stop the program
                break
            
        self.cam2.release()



    def stereoCameraframePerSecondCalculation(self, runTimeInSec):
        # This is the function that uses threading without barrier.
        # The purpose of this function is to find the maximum period of cameras which used for live streaming
        # Since we are not using barrier, both camera are not exactly syncronize, which implies that cameras do not capture images at the same time.
        timeStampCam1 = []
        timeStampCam2 = []
        period = []

        csv_Cam1 = 'Camera1_TimeCounter'
        csv_Cam2 = 'Camera2_TimeCounter'

        # Call the function to create the folder
        created_folder = self.create_folder()

        stopThread = threading.Event()

        os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
        # CSV file initialization
        with open(csv_Cam1, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['FrameCounter', 'Timestamp (s)'])

        with open(csv_Cam2, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['FrameCounter', 'Timestamp (s)'])

        # Start threads for each camera
        threadCam1 = threading.Thread(target=self.saveImageCounterAndTimeToCsv_Cam1, args = (csv_Cam1, created_folder,stopThread))
        threadCam2 = threading.Thread(target=self.saveImageCounterAndTimeToCsv_Cam2, args = (csv_Cam2, created_folder,stopThread))

        
        # Start threads
        print('Thread start')
        threadCam1.start()
        threadCam2.start()

        time.sleep(runTimeInSec)
        stopThread.set()

        # Wait for threads to finish
        threadCam1.join()
        threadCam2.join()

        # Read the CSV file and extract Timestamp values
        with open(csv_Cam1, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                frame_counter, timestamp = row
                timeStampCam1.append(float(timestamp))  # Assuming Timestamp values are stored as strings

        with open(csv_Cam2, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                frame_counter, timestamp = row
                timeStampCam2.append(float(timestamp))  # Assuming Timestamp values are stored as strings

        for i in range(len(timeStampCam1)-1):
            period.append(timeStampCam1[i+1]-timeStampCam1[i])

        for i in range(len(timeStampCam2)-1):
            period.append(timeStampCam2[i+1]-timeStampCam2[i])

        maximumPeriod = max(period)
        minimumPeriod = min(period)

        stereoPeriodInSec = (maximumPeriod+minimumPeriod)/2

        return stereoPeriodInSec
    
    def addHSVFilter(self, image1, image2):
        
        csv_Cam1 = 'maskImg1'
        csv_Cam2 = 'maskImg2'

        os.chdir(self.directory)
        if not os.path.exists(self.folderNameHSVValue):
            print(f"Folder '{self.folderNameHSVValue}' does not exists")

        os.chdir(self.directory + '/' + self.folderNameHSVValue )
        with open(csv_Cam1, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:

                kernel_Cleaning_Img1,kernel_Closing_Img1,lower_hMin_Img1,lower_sMin_Img1,lower_vMin_Img1,lower_hMax_Img1,lower_sMax_Img1,lower_vMax_Img1,upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1,upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1 = row

        # Read the CSV file and extract Timestamp values
        os.chdir(self.directory + '/' + self.folderNameHSVValue )
        with open(csv_Cam2, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                kernel_Cleaning_Img2,kernel_Closing_Img2,lower_hMin_Img2,lower_sMin_Img2,lower_vMin_Img2,lower_hMax_Img2,lower_sMax_Img2,lower_vMax_Img2,upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2,upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2 = row

        #Image 1 kernel: Defines the neighborhood around each pixel that is considered during the operation
        kernelClosingImg1 = np.ones((int(kernel_Closing_Img1),int(kernel_Closing_Img1)), np.uint8)
        kernelCleaningImg1 = np.ones((int(kernel_Cleaning_Img1),int(kernel_Cleaning_Img1)), np.uint8)

        #Image 2 kernel: Defines the neighborhood around each pixel that is considered during the operation
        kernelClosingImg2 = np.ones((int(kernel_Closing_Img2),int(kernel_Closing_Img2)), np.uint8)
        kernelCleaningImg2 = np.ones((int(kernel_Cleaning_Img2),int(kernel_Cleaning_Img2)), np.uint8)

        # Blurring the frame
        blur1 = cv2.GaussianBlur(image1,(5,5),0) 
        blur2 = cv2.GaussianBlur(image2,(5,5),0)

        hsvImg1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
        hsvImg2 = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)
        
        #Img1
        # lower boundary RED color range values; Hue (0 - 10)
        lower1Img1 = np.array([int(lower_hMin_Img1), int(lower_sMin_Img1), int(lower_vMin_Img1)])
        upper1Img1 = np.array([int(lower_hMax_Img1), int(lower_sMax_Img1), int(lower_vMax_Img1)])
        # upper boundary RED color range values; Hue (160 - 180)
        lower2Img1 = np.array([int(upper_hMin_Img1),int(upper_sMin_Img1),int(upper_vMin_Img1)])
        upper2Img1 = np.array([int(upper_hMax_Img1),int(upper_sMax_Img1),int(upper_vMax_Img1)])

        lower_mask_Img1 = cv2.inRange(hsvImg1, lower1Img1, upper1Img1)
        upper_mask_Img1 = cv2.inRange(hsvImg1, lower2Img1, upper2Img1)
        full_mask_Img1 = lower_mask_Img1 + upper_mask_Img1
        #maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)

        #Img2
        # lower boundary RED color range values; Hue (0 - 10)
        lower1Img2 = np.array([int(lower_hMin_Img2), int(lower_sMin_Img2), int(lower_vMin_Img2)])
        upper1Img2 = np.array([int(lower_hMax_Img2), int(lower_sMax_Img2), int(lower_vMax_Img2)])
        # upper boundary RED color range values; Hue (160 - 180)
        lower2Img2 = np.array([int(upper_hMin_Img2),int(upper_sMin_Img2),int(upper_vMin_Img2)])
        upper2Img2 = np.array([int(upper_hMax_Img2),int(upper_sMax_Img2),int(upper_vMax_Img2)])
        
        lower_mask_Img2 = cv2.inRange(hsvImg2, lower1Img2, upper1Img2)
        upper_mask_Img2 = cv2.inRange(hsvImg2, lower2Img2, upper2Img2)
        full_mask_Img2 = lower_mask_Img2 + upper_mask_Img2
        #maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)

    
        #Alternative approach for mask creation for red color
        # define range of red color in HSV
        #lowerLimit1 = np.array([170,150,20])
        #upperLimit1 = np.array([180,255,255])

        #lowerLimit2 = np.array([170,150,50])
        #upperLimit2 = np.array([180,255,255])

        #maskImg1 = cv2.inRange(hsvImg1, lowerLimit1, upperLimit1)
        #maskImg2 = cv2.inRange(hsvImg2, lowerLimit2, upperLimit2)

        # morphological operation which is a combination of erosion followed by dilation
        full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_OPEN, kernelCleaningImg1)
        full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_OPEN, kernelCleaningImg2)
        
        # morphological operation which is a combination of dilation followed by erosion
        full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_CLOSE, kernelClosingImg1)
        full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_CLOSE, kernelClosingImg2)
        
        return full_mask_Img1, full_mask_Img2
    
    def findCentroidOfMarkers(self, image1, image2, full_mask_Img1, full_mask_Img2):
        
        cXImg1 = []
        cYImg1 = []
        sorted_cXImg1 = []
        sorted_cYImg1 = []

        cXImg2 = []
        cYImg2 = []
        sorted_cXImg2 = []
        sorted_cYImg2 = []

        image_1 = image1.copy()
        image_2 = image2.copy()
        #maskImg1 = cv2.bitwise_and(image1.copy(), image1.copy(), mask=full_mask_Img1.copy())
        #maskImg2 = cv2.bitwise_and(image2.copy(), image2.copy(), mask=full_mask_Img2.copy())
        
        cntsImg1 = cv2.findContours(full_mask_Img1.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cntsImg1 = imutils.grab_contours(cntsImg1)

        cntsImg2 = cv2.findContours(full_mask_Img2.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cntsImg2 = imutils.grab_contours(cntsImg2)

        markerNumImg1 = 0
        for c in cntsImg1:
        
            #area = cv2.contourArea(c)
            #cv2.drawContours(img1, [c], -1, (255,255,0), 1)

            # moments of a contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
            else:
                cx, cy = 0, 0

            cv2.circle(image_1, (cx,cy), 3, (255,255,255), -1)
            cXImg1.append(cx)
            cYImg1.append(cy)
            markerNumImg1+=1


        markerNumImg2 = 0
        for c in cntsImg2:

            #area = cv2.contourArea(c)
            #cv2.drawContours(img2, [c], -1, (255,255,0), 1)

            # moments of a contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
            else:
                cx, cy = 0, 0

            cv2.circle(image_2, (cx,cy), 3, (255,255,255), -1)

            cXImg2.append(cx)
            cYImg2.append(cy)
            markerNumImg2+=1

        sortedIndices_cYImg1 = np.argsort(cYImg1)
        sortedIndices_cYImg2 = np.argsort(cYImg2)
        #print('markerNumImg1:',markerNumImg1)
        #print('markerNumImg2:',markerNumImg2)


        for i in sortedIndices_cYImg1:
            sorted_cXImg1.append(cXImg1[i])
            sorted_cYImg1.append(cYImg1[i])

        for i in sortedIndices_cYImg2:
            sorted_cXImg2.append(cXImg2[i])
            sorted_cYImg2.append(cYImg2[i])

        return image_1, image_2, sorted_cXImg1, sorted_cYImg1, sorted_cXImg2, sorted_cYImg2, markerNumImg1, markerNumImg2
    
    def callback(self,x):
        pass

    def setHSVFilter(self, imageReading = False, liveStreaming = True, WindowName_Image_CentroidOfMarkers = 'Original image whith centroid of markers (Image 1 and 2)', windowCam1 = 'HSV filter of Camera 1', windowCam2 = 'HSV filter of Camera 2', imageFolderName = '2024-00-00_00-00-00', counter = 0):

        csv_Cam1 = 'maskImg1'
        csv_Cam2 = 'maskImg2'
        
        

        if liveStreaming == True:

            os.chdir(self.directory)
            
            if not os.path.exists(self.folderNameHSVValue):
                os.makedirs(self.folderNameHSVValue)
                print(f"Folder '{self.folderNameHSVValue}' created successfully")

                os.chdir(self.directory + '/' + self.folderNameHSVValue )
                # CSV file initialization
                with open(csv_Cam1, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Kernel value for cleaning_Img1', 'Kernel value for closing_Img1', 
                                        'Lower boundary of color_HMin_Img1', 'Lower boundary of color_SMin_Img1','Lower boundary of color_VMin_Img1'
                                        'Lower boundary of color_HMax_Img1','Lower boundary of color_SMax_Img1','Lower boundary of color_VMax_Img1'
                                        'Upper boundary of color_HMin_Img1','Upper boundary of color_SMin_Img1','Upper boundary of color_VMin_Img1'
                                        'Upper boundary of color_HMax_Img1','Upper boundary of color_SMax_Img1','Upper boundary of color_VMax_Img1'])
                    
                with open(csv_Cam2, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Kernel value for cleaning_Img2', 'Kernel value for closing_Img2', 
                                        'Lower boundary of color_HMin_Img2', 'Lower boundary of color_SMin_Img2','Lower boundary of color_VMin_Img2'
                                        'Lower boundary of color_HMax_Img2','Lower boundary of color_SMax_Img2','Lower boundary of color_VMax_Img2'
                                        'Upper boundary of color_HMin_Img2','Upper boundary of color_SMin_Img2','Upper boundary of color_VMin_Img2'
                                        'Upper boundary of color_HMax_Img2','Upper boundary of color_SMax_Img2','Upper boundary of color_VMax_Img2'])
                    
                cv2.namedWindow(windowCam1)

                #create trackbar for color change
                cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,0,20,self.callback)
                cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,0,20,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,0,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,12,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,150,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,180,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                cv2.namedWindow(windowCam2)

                #create trackbar for color change
                cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,0,20,self.callback)
                cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,0,20,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,0,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,20,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,165,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,180,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

            else:
                print(f"Folder '{self.folderNameHSVValue}' already exists")

                os.chdir(self.directory + '/' + self.folderNameHSVValue )
                
                # Read the CSV file and extract Timestamp values
                
                with open(csv_Cam1, 'r') as csvfile:
                    csv_reader = csv.reader(csvfile)
    
                    # Skip the header row
                    next(csv_reader)
                    
                    # Iterate through rows and extract Timestamp values
                    boolCam1 = 0
                    for row in csv_reader:
                        boolCam1 = 1
                        kernel_Cleaning_Img1,kernel_Closing_Img1,lower_hMin_Img1,lower_sMin_Img1,lower_vMin_Img1,lower_hMax_Img1,lower_sMax_Img1,lower_vMax_Img1,upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1,upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1 = row

                # Read the CSV file and extract Timestamp values
                with open(csv_Cam2, 'r') as csvfile:
                    csv_reader = csv.reader(csvfile)
                    
                    # Skip the header row
                    next(csv_reader)
                    
                    # Iterate through rows and extract Timestamp values
                    boolCam2 = 0
                    for row in csv_reader:
                        boolCam2 = 1
                        kernel_Cleaning_Img2,kernel_Closing_Img2,lower_hMin_Img2,lower_sMin_Img2,lower_vMin_Img2,lower_hMax_Img2,lower_sMax_Img2,lower_vMax_Img2,upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2,upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2 = row

                    if boolCam1 == 1 and boolCam2 == 1:
                        cv2.namedWindow(windowCam1)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,int(kernel_Cleaning_Img1),20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,int(kernel_Closing_Img1),20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,int(lower_hMin_Img1),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,int(lower_sMin_Img1),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,int(lower_vMin_Img1),255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,int(lower_hMax_Img1),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,int(lower_sMax_Img1),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,int(lower_vMax_Img1),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,int(upper_hMin_Img1),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,int(upper_sMin_Img1),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,int(upper_vMin_Img1),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,int(upper_hMax_Img1),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,int(upper_sMax_Img1),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,int(upper_vMax_Img1),255,self.callback)

                        cv2.namedWindow(windowCam2)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,int(kernel_Cleaning_Img2),20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,int(kernel_Closing_Img2),20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,int(lower_hMin_Img2),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,int(lower_sMin_Img2),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,int(lower_vMin_Img2),255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,int(lower_hMax_Img2),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,int(lower_sMax_Img2),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,int(lower_vMax_Img2),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,int(upper_hMin_Img2),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,int(upper_sMin_Img2),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,int(upper_vMin_Img2),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,int(upper_hMax_Img2),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,int(upper_sMax_Img2),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,int(upper_vMax_Img2),255,self.callback)

                        os.chdir(self.directory + '/' + self.folderNameHSVValue )
                        # CSV file initialization
                        with open(csv_Cam1, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['Kernel value for cleaning_Img1', 'Kernel value for closing_Img1', 
                                                'Lower boundary of color_HMin_Img1', 'Lower boundary of color_SMin_Img1','Lower boundary of color_VMin_Img1'
                                                'Lower boundary of color_HMax_Img1','Lower boundary of color_SMax_Img1','Lower boundary of color_VMax_Img1'
                                                'Upper boundary of color_HMin_Img1','Upper boundary of color_SMin_Img1','Upper boundary of color_VMin_Img1'
                                                'Upper boundary of color_HMax_Img1','Upper boundary of color_SMax_Img1','Upper boundary of color_VMax_Img1'])
                            
                        with open(csv_Cam2, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['Kernel value for cleaning_Img2', 'Kernel value for closing_Img2', 
                                                'Lower boundary of color_HMin_Img2', 'Lower boundary of color_SMin_Img2','Lower boundary of color_VMin_Img2'
                                                'Lower boundary of color_HMax_Img2','Lower boundary of color_SMax_Img2','Lower boundary of color_VMax_Img2'
                                                'Upper boundary of color_HMin_Img2','Upper boundary of color_SMin_Img2','Upper boundary of color_VMin_Img2'
                                                'Upper boundary of color_HMax_Img2','Upper boundary of color_SMax_Img2','Upper boundary of color_VMax_Img2'])
                    else:
                        cv2.namedWindow(windowCam1)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,0,20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,0,20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,0,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,12,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,150,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,180,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                        cv2.namedWindow(windowCam2)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,0,20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,0,20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,0,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,20,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,165,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,180,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

            while self.cam1.isOpened() and self.cam2.isOpened():
                        

                        succes1, img1 = self.cam1.read() # read returns tuple, so we need to write it like success1, img. Otherwise, we will face the error: mat is not a numerical tuple
                        succes2, img2 = self.cam2.read()

                        # Blurring the frame
                        blur1 = cv2.GaussianBlur(img1,(5,5),0) 
                        blur2 = cv2.GaussianBlur(img2,(5,5),0)

                        hsvImg1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
                        hsvImg2 = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)

                        #Img1
                        # lower boundary RED color range values; Hue (0 - 10)
                        lower_hMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_HMin_Img1',windowCam1) #get the current slider position
                        lower_sMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_SMin_Img1',windowCam1)
                        lower_vMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_VMin_Img1',windowCam1)

                        lower_hMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_HMax_Img1',windowCam1)
                        lower_sMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_SMax_Img1',windowCam1)
                        lower_vMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_VMax_Img1',windowCam1)
                                        
                        lower1Img1 = np.array([lower_hMin_Img1, lower_sMin_Img1, lower_vMin_Img1])
                        upper1Img1 = np.array([lower_hMax_Img1, lower_sMax_Img1, lower_vMax_Img1])

                        # upper boundary RED color range values; Hue (160 - 180)
                        upper_hMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_HMin_Img1',windowCam1) #get the current slider position
                        upper_sMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_SMin_Img1',windowCam1)
                        upper_vMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_VMin_Img1',windowCam1)

                        upper_hMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_HMax_Img1',windowCam1)
                        upper_sMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_SMax_Img1',windowCam1)
                        upper_vMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_VMax_Img1',windowCam1)

                        lower2Img1 = np.array([upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1])
                        upper2Img1 = np.array([upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1])

                        lower_mask_Img1 = cv2.inRange(hsvImg1, lower1Img1, upper1Img1)
                        upper_mask_Img1 = cv2.inRange(hsvImg1, lower2Img1, upper2Img1)
                        full_mask_Img1 = lower_mask_Img1 + upper_mask_Img1
                        #maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)

                        #Img2
                        # lower boundary RED color range values; Hue (0 - 10)
                        lower_hMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_HMin_Img2',windowCam2) #get the current slider position
                        lower_sMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_SMin_Img2',windowCam2)
                        lower_vMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_VMin_Img2',windowCam2)

                        lower_hMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_HMax_Img2',windowCam2)
                        lower_sMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_SMax_Img2',windowCam2)
                        lower_vMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_VMax_Img2',windowCam2)
                                        
                        lower1Img2 = np.array([lower_hMin_Img2, lower_sMin_Img2, lower_vMin_Img2])
                        upper1Img2 = np.array([lower_hMax_Img2, lower_sMax_Img2, lower_vMax_Img2])

                        # upper boundary RED color range values; Hue (160 - 180)
                        upper_hMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_HMin_Img2',windowCam2) #get the current slider position
                        upper_sMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_SMin_Img2',windowCam2)
                        upper_vMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_VMin_Img2',windowCam2)

                        upper_hMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_HMax_Img2',windowCam2)
                        upper_sMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_SMax_Img2',windowCam2)
                        upper_vMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_VMax_Img2',windowCam2)

                        lower2Img2 = np.array([upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2])
                        upper2Img2 = np.array([upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2])
                        
                        lower_mask_Img2 = cv2.inRange(hsvImg2, lower1Img2, upper1Img2)
                        upper_mask_Img2 = cv2.inRange(hsvImg2, lower2Img2, upper2Img2)
                        full_mask_Img2 = lower_mask_Img2 + upper_mask_Img2
                        #maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)

                        kernel_Cleaning_Img1 = cv2.getTrackbarPos('Kernel value for cleaning_Img1',windowCam1) #get the current slider position
                        kernel_Cleaning_Img2 = cv2.getTrackbarPos('Kernel value for cleaning_Img2',windowCam2) #get the current slider position
                        
                        kernel_Closing_Img1 = cv2.getTrackbarPos('Kernel value for cleaning_Img1',windowCam1) #get the current slider position
                        kernel_Closing_Img2 = cv2.getTrackbarPos('Kernel value for cleaning_Img2',windowCam2) #get the current slider position
    
                        #Image 1 kernel: Defines the neighborhood around each pixel that is considered during the operation
                        kernelClosingImg1 = np.ones((kernel_Closing_Img1,kernel_Closing_Img1), np.uint8)
                        kernelCleaningImg1 = np.ones((kernel_Cleaning_Img1,kernel_Cleaning_Img1), np.uint8)

                        #Image 2 kernel: Defines the neighborhood around each pixel that is considered during the operation
                        kernelClosingImg2 = np.ones((kernel_Closing_Img2,kernel_Closing_Img2), np.uint8)
                        kernelCleaningImg2 = np.ones((kernel_Cleaning_Img2,kernel_Cleaning_Img2), np.uint8)
                        
                        # morphological operation which is a combination of erosion followed by dilation
                        full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_OPEN, kernelCleaningImg1)
                        full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_OPEN, kernelCleaningImg2)
                        
                        # morphological operation which is a combination of dilation followed by erosion
                        full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_CLOSE, kernelClosingImg1)
                        full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_CLOSE, kernelClosingImg2)

                        k = cv2.waitKey(1)

                        if k == 27: #Esc key to stop the program
                            break
                        
                        maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)
                        maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)
                        image_1, image_2, sorted_cXImg1, sorted_cYImg1, sorted_cXImg2, sorted_cYImg2, markerNumImg1, markerNumImg2 = self.findCentroidOfMarkers(img1, img2, full_mask_Img1, full_mask_Img2)
                        cv2.imshow(windowCam1,maskImg1)
                        cv2.imshow(windowCam2,maskImg2)

                        image2 = cv2.hconcat([image_1, image_2])
                        cv2.imshow(WindowName_Image_CentroidOfMarkers,image2)

            print('here')
            os.chdir(self.directory + '/' + self.folderNameHSVValue )            
            with open(csv_Cam1, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([kernel_Cleaning_Img1,kernel_Closing_Img1,
                                        lower_hMin_Img1,lower_sMin_Img1,lower_vMin_Img1,
                                        lower_hMax_Img1,lower_sMax_Img1,lower_vMax_Img1,
                                        upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1,
                                        upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1])

            with open(csv_Cam2, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([kernel_Cleaning_Img2,kernel_Closing_Img2,
                                        lower_hMin_Img2,lower_sMin_Img2,lower_vMin_Img2,
                                        lower_hMax_Img2,lower_sMax_Img2,lower_vMax_Img2,
                                        upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2,
                                        upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2])       
            self.cam1.release()
            self.cam2.release()
            cv2.destroyAllWindows()

        if imageReading == True:

            os.chdir(self.directory)
            if not os.path.exists(self.folderNameHSVValue):
                os.makedirs(self.folderNameHSVValue)
                print(f"Folder '{self.folderNameHSVValue}' created successfully")

                os.chdir(self.directory + '/' + self.folderNameHSVValue )
                # CSV file initialization
                with open(csv_Cam1, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Kernel value for cleaning_Img1', 'Kernel value for closing_Img1', 
                                        'Lower boundary of color_HMin_Img1', 'Lower boundary of color_SMin_Img1','Lower boundary of color_VMin_Img1'
                                        'Lower boundary of color_HMax_Img1','Lower boundary of color_SMax_Img1','Lower boundary of color_VMax_Img1'
                                        'Upper boundary of color_HMin_Img1','Upper boundary of color_SMin_Img1','Upper boundary of color_VMin_Img1'
                                        'Upper boundary of color_HMax_Img1','Upper boundary of color_SMax_Img1','Upper boundary of color_VMax_Img1'])
                    
                with open(csv_Cam2, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Kernel value for cleaning_Img2', 'Kernel value for closing_Img2', 
                                        'Lower boundary of color_HMin_Img2', 'Lower boundary of color_SMin_Img2','Lower boundary of color_VMin_Img2'
                                        'Lower boundary of color_HMax_Img2','Lower boundary of color_SMax_Img2','Lower boundary of color_VMax_Img2'
                                        'Upper boundary of color_HMin_Img2','Upper boundary of color_SMin_Img2','Upper boundary of color_VMin_Img2'
                                        'Upper boundary of color_HMax_Img2','Upper boundary of color_SMax_Img2','Upper boundary of color_VMax_Img2'])
                    
                cv2.namedWindow(windowCam1)

                #create trackbar for color change
                cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,0,20,self.callback)
                cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,0,20,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,0,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,12,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,150,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,180,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                cv2.namedWindow(windowCam2)

                #create trackbar for color change
                cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,0,20,self.callback)
                cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,0,20,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,0,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,20,180,self.callback)
                cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,165,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,180,180,self.callback)
                cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

            else:
                print(f"Folder '{self.folderNameHSVValue}' already exists")

                os.chdir(self.directory + '/' + self.folderNameHSVValue )
                
                # Read the CSV file and extract Timestamp values
                with open(csv_Cam1, 'r') as csvfile:
                    csv_reader = csv.reader(csvfile)

                    # Skip the header row
                   
                    next(csv_reader)
                   
                    # Iterate through rows and extract Timestamp values
                    boolCam1=0
                    for row in csv_reader:
                        boolCam1=1
                        kernel_Cleaning_Img1,kernel_Closing_Img1,lower_hMin_Img1,lower_sMin_Img1,lower_vMin_Img1,lower_hMax_Img1,lower_sMax_Img1,lower_vMax_Img1,upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1,upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1 = row
                    
                # Read the CSV file and extract Timestamp values
                with open(csv_Cam2, 'r') as csvfile:
                    csv_reader = csv.reader(csvfile)
                    
                    # Skip the header row
                    next(csv_reader)
                    
                    # Iterate through rows and extract Timestamp values
                    boolCam2=0
                    for row in csv_reader:
                        boolCam2=1
                        kernel_Cleaning_Img2,kernel_Closing_Img2,lower_hMin_Img2,lower_sMin_Img2,lower_vMin_Img2,lower_hMax_Img2,lower_sMax_Img2,lower_vMax_Img2,upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2,upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2 = row

                    if boolCam1 == 1 and boolCam2 == 1:
                        cv2.namedWindow(windowCam1)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,int(kernel_Cleaning_Img1),20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,int(kernel_Closing_Img1),20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,int(lower_hMin_Img1),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,int(lower_sMin_Img1),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,int(lower_vMin_Img1),255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,int(lower_hMax_Img1),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,int(lower_sMax_Img1),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,int(lower_vMax_Img1),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,int(upper_hMin_Img1),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,int(upper_sMin_Img1),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,int(upper_vMin_Img1),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,int(upper_hMax_Img1),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,int(upper_sMax_Img1),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,int(upper_vMax_Img1),255,self.callback)

                        cv2.namedWindow(windowCam2)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,int(kernel_Cleaning_Img2),20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,int(kernel_Closing_Img2),20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,int(lower_hMin_Img2),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,int(lower_sMin_Img2),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,int(lower_vMin_Img2),255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,int(lower_hMax_Img2),180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,int(lower_sMax_Img2),255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,int(lower_vMax_Img2),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,int(upper_hMin_Img2),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,int(upper_sMin_Img2),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,int(upper_vMin_Img2),255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,int(upper_hMax_Img2),180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,int(upper_sMax_Img2),255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,int(upper_vMax_Img2),255,self.callback)

                        os.chdir(self.directory + '/' + self.folderNameHSVValue )
                        # CSV file initialization
                        with open(csv_Cam1, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['Kernel value for cleaning_Img1', 'Kernel value for closing_Img1', 
                                                'Lower boundary of color_HMin_Img1', 'Lower boundary of color_SMin_Img1','Lower boundary of color_VMin_Img1'
                                                'Lower boundary of color_HMax_Img1','Lower boundary of color_SMax_Img1','Lower boundary of color_VMax_Img1'
                                                'Upper boundary of color_HMin_Img1','Upper boundary of color_SMin_Img1','Upper boundary of color_VMin_Img1'
                                                'Upper boundary of color_HMax_Img1','Upper boundary of color_SMax_Img1','Upper boundary of color_VMax_Img1'])
                            
                        with open(csv_Cam2, 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow(['Kernel value for cleaning_Img2', 'Kernel value for closing_Img2', 
                                                'Lower boundary of color_HMin_Img2', 'Lower boundary of color_SMin_Img2','Lower boundary of color_VMin_Img2'
                                                'Lower boundary of color_HMax_Img2','Lower boundary of color_SMax_Img2','Lower boundary of color_VMax_Img2'
                                                'Upper boundary of color_HMin_Img2','Upper boundary of color_SMin_Img2','Upper boundary of color_VMin_Img2'
                                                'Upper boundary of color_HMax_Img2','Upper boundary of color_SMax_Img2','Upper boundary of color_VMax_Img2'])
                    else:
                        cv2.namedWindow(windowCam1)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img1',windowCam1,0,20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img1',windowCam1,0,20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img1',windowCam1,0,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img1',windowCam1,12,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img1',windowCam1,150,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img1',windowCam1,100,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img1',windowCam1,20,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img1',windowCam1,180,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img1',windowCam1,255,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img1',windowCam1,255,255,self.callback)

                        cv2.namedWindow(windowCam2)

                        #create trackbar for color change
                        cv2.createTrackbar('Kernel value for cleaning_Img2',windowCam2,0,20,self.callback)
                        cv2.createTrackbar('Kernel value for closing_Img2',windowCam2,0,20,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMin_Img2',windowCam2,0,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                        cv2.createTrackbar('Lower boundary of color_HMax_Img2',windowCam2,20,180,self.callback)
                        cv2.createTrackbar('Lower boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                        cv2.createTrackbar('Lower boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMin_Img2',windowCam2,165,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMin_Img2',windowCam2,100,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMin_Img2',windowCam2,20,255,self.callback)

                        cv2.createTrackbar('Upper boundary of color_HMax_Img2',windowCam2,180,180,self.callback)
                        cv2.createTrackbar('Upper boundary of color_SMax_Img2',windowCam2,255,255,self.callback)
                        cv2.createTrackbar('Upper boundary of color_VMax_Img2',windowCam2,255,255,self.callback)

            #Load Image of Cam1
            
            img1 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera1/' + '/image1_' + str(counter) + '.png')

            #Load Image of Cam2
            img2 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera2/' + '/image2_' + str(counter) + '.png')

            img1, img2 = self.undistortRectify(img1, img2, 'stereoCameraCalibration')

            while True:
                # Blurring the frame
                blur1 = cv2.GaussianBlur(img1,(5,5),0) 
                blur2 = cv2.GaussianBlur(img2,(5,5),0)

                hsvImg1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
                hsvImg2 = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)

                #Img1
                # lower boundary RED color range values; Hue (0 - 10)
                lower_hMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_HMin_Img1',windowCam1) #get the current slider position
                lower_sMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_SMin_Img1',windowCam1)
                lower_vMin_Img1 = cv2.getTrackbarPos('Lower boundary of color_VMin_Img1',windowCam1)

                lower_hMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_HMax_Img1',windowCam1)
                lower_sMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_SMax_Img1',windowCam1)
                lower_vMax_Img1 = cv2.getTrackbarPos('Lower boundary of color_VMax_Img1',windowCam1)
                                
                lower1Img1 = np.array([int(lower_hMin_Img1), int(lower_sMin_Img1), int(lower_vMin_Img1)])
                upper1Img1 = np.array([int(lower_hMax_Img1), int(lower_sMax_Img1), int(lower_vMax_Img1)])

                # upper boundary RED color range values; Hue (160 - 180)
                upper_hMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_HMin_Img1',windowCam1) #get the current slider position
                upper_sMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_SMin_Img1',windowCam1)
                upper_vMin_Img1 = cv2.getTrackbarPos('Upper boundary of color_VMin_Img1',windowCam1)

                upper_hMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_HMax_Img1',windowCam1)
                upper_sMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_SMax_Img1',windowCam1)
                upper_vMax_Img1 = cv2.getTrackbarPos('Upper boundary of color_VMax_Img1',windowCam1)

                lower2Img1 = np.array([int(upper_hMin_Img1),int(upper_sMin_Img1),int(upper_vMin_Img1)])
                upper2Img1 = np.array([int(upper_hMax_Img1),int(upper_sMax_Img1),int(upper_vMax_Img1)])

                lower_mask_Img1 = cv2.inRange(hsvImg1, lower1Img1, upper1Img1)
                upper_mask_Img1 = cv2.inRange(hsvImg1, lower2Img1, upper2Img1)
                full_mask_Img1 = lower_mask_Img1 + upper_mask_Img1
                #maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)

                #Img2
                # lower boundary RED color range values; Hue (0 - 10)
                lower_hMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_HMin_Img2',windowCam2) #get the current slider position
                lower_sMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_SMin_Img2',windowCam2)
                lower_vMin_Img2 = cv2.getTrackbarPos('Lower boundary of color_VMin_Img2',windowCam2)

                lower_hMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_HMax_Img2',windowCam2)
                lower_sMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_SMax_Img2',windowCam2)
                lower_vMax_Img2 = cv2.getTrackbarPos('Lower boundary of color_VMax_Img2',windowCam2)
                                
                lower1Img2 = np.array([int(lower_hMin_Img2), int(lower_sMin_Img2), int(lower_vMin_Img2)])
                upper1Img2 = np.array([int(lower_hMax_Img2), int(lower_sMax_Img2), int(lower_vMax_Img2)])

                # upper boundary RED color range values; Hue (160 - 180)
                upper_hMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_HMin_Img2',windowCam2) #get the current slider position
                upper_sMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_SMin_Img2',windowCam2)
                upper_vMin_Img2 = cv2.getTrackbarPos('Upper boundary of color_VMin_Img2',windowCam2)

                upper_hMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_HMax_Img2',windowCam2)
                upper_sMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_SMax_Img2',windowCam2)
                upper_vMax_Img2 = cv2.getTrackbarPos('Upper boundary of color_VMax_Img2',windowCam2)

                lower2Img2 = np.array([int(upper_hMin_Img2),int(upper_sMin_Img2),int(upper_vMin_Img2)])
                upper2Img2 = np.array([int(upper_hMax_Img2),int(upper_sMax_Img2),int(upper_vMax_Img2)])
                
                lower_mask_Img2 = cv2.inRange(hsvImg2, lower1Img2, upper1Img2)
                upper_mask_Img2 = cv2.inRange(hsvImg2, lower2Img2, upper2Img2)
                full_mask_Img2 = lower_mask_Img2 + upper_mask_Img2
                #maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)

                kernel_Cleaning_Img1 = cv2.getTrackbarPos('Kernel value for cleaning_Img1',windowCam1) #get the current slider position
                kernel_Cleaning_Img2 = cv2.getTrackbarPos('Kernel value for cleaning_Img2',windowCam2) #get the current slider position
                
                kernel_Closing_Img1 = cv2.getTrackbarPos('Kernel value for cleaning_Img1',windowCam1) #get the current slider position
                kernel_Closing_Img2 = cv2.getTrackbarPos('Kernel value for cleaning_Img2',windowCam2) #get the current slider position

                #Image 1 kernel: Defines the neighborhood around each pixel that is considered during the operation
                kernelClosingImg1 = np.ones((int(kernel_Closing_Img1),int(kernel_Closing_Img1)), np.uint8)
                kernelCleaningImg1 = np.ones((int(kernel_Cleaning_Img1),int(kernel_Cleaning_Img1)), np.uint8)

                #Image 2 kernel: Defines the neighborhood around each pixel that is considered during the operation
                kernelClosingImg2 = np.ones((int(kernel_Closing_Img2),int(kernel_Closing_Img2)), np.uint8)
                kernelCleaningImg2 = np.ones((int(kernel_Cleaning_Img2),int(kernel_Cleaning_Img2)), np.uint8)
                
                # morphological operation which is a combination of erosion followed by dilation
                full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_OPEN, kernelCleaningImg1)
                full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_OPEN, kernelCleaningImg2)
                
                # morphological operation which is a combination of dilation followed by erosion
                full_mask_Img1 = cv2.morphologyEx(full_mask_Img1, cv2.MORPH_CLOSE, kernelClosingImg1)
                full_mask_Img2 = cv2.morphologyEx(full_mask_Img2, cv2.MORPH_CLOSE, kernelClosingImg2)

                k = cv2.waitKey(1)

                if k == 27: #Esc key to stop the program
                    break
                
                maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)
                maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)
                image_1, image_2, sorted_cXImg1, sorted_cYImg1, sorted_cXImg2, sorted_cYImg2, markerNumImg1, markerNumImg2 = self.findCentroidOfMarkers(img1, img2, full_mask_Img1, full_mask_Img2)

                cv2.imshow(windowCam1,maskImg1)
                cv2.imshow(windowCam2,maskImg2)

                image2 = cv2.hconcat([image_1, image_2])
                cv2.imshow(WindowName_Image_CentroidOfMarkers,image2)
             
            os.chdir(self.directory + '/' + self.folderNameHSVValue )  
            with open(csv_Cam1, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([kernel_Cleaning_Img1,kernel_Closing_Img1,
                                        lower_hMin_Img1,lower_sMin_Img1,lower_vMin_Img1,
                                        lower_hMax_Img1,lower_sMax_Img1,lower_vMax_Img1,
                                        upper_hMin_Img1,upper_sMin_Img1,upper_vMin_Img1,
                                        upper_hMax_Img1,upper_sMax_Img1,upper_vMax_Img1])

            with open(csv_Cam2, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([kernel_Cleaning_Img2,kernel_Closing_Img2,
                                        lower_hMin_Img2,lower_sMin_Img2,lower_vMin_Img2,
                                        lower_hMax_Img2,lower_sMax_Img2,lower_vMax_Img2,
                                        upper_hMin_Img2,upper_sMin_Img2,upper_vMin_Img2,
                                        upper_hMax_Img2,upper_sMax_Img2,upper_vMax_Img2])       

            cv2.destroyAllWindows()

    def centroidOfCDMMarkerExtraction_AllFrames(self, liveStreaming, readImage, imageFolderName):

        windowName1 = 'Image (1 and 2) with marker centroids'
        windowName2 = 'HSV filter for image 1 and 2'

        csv_Cam1and2 = 'centroidOfMarkers.csv'

        csv_numOfFrames_Cam1 = 'numOfFramesCam1.csv'
        csv_numOfFrames_Cam2 = 'numOfFramesCam2.csv'
        csv_imageCounterMoreMarkerDetected = 'imageCounterMoreMarkerDetected.csv'

        #Image 1 kernel: Defines the neighborhood around each pixel that is considered during the operation
        #kernelClosingImg1 = np.ones((3,3), np.uint8)
        #kernelCleaningImg1 = np.ones((3,3), np.uint8)

        #Image 2 kernel: Defines the neighborhood around each pixel that is considered during the operation
        #kernelClosingImg2 = np.ones((3,3), np.uint8)
        #kernelCleaningImg2 = np.ones((3,3), np.uint8)

        counter = 0
        if liveStreaming == True:
            # Call the function to create the folder
            if self.cam1.isOpened()==False or self.cam2.isOpened()==False:

                print('Error for openning cameras')

            else:
                # Call the function to create the folder
                created_folder = self.create_folder()

                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                # CSV file initialization
                with open(csv_Cam1and2, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['FrameCounterImg1', 'Number of Markers In Img1', 'cXImg1', 'cYImg1', 'FrameCounterImg2', 'Number of Markers In Img2', 'cXImg2', 'cYImg2'])    
               
                print('Press Esc to exit')
                while self.cam1.isOpened() and self.cam2.isOpened():

                    succes1, img1 = self.cam1.read() # read returns tuple, so we need to write it like success1, img. Otherwise, we will face the error: mat is not a numerical tuple
                    succes2, img2 = self.cam2.read()

                    img1, img2 = self.undistortRectify(img1, img2, 'stereoCameraCalibration')

                    full_mask_Img1, full_mask_Img2 = self.addHSVFilter(img1, img2)

                    image_1, image_2, sorted_cXImg1, sorted_cYImg1, sorted_cXImg2, sorted_cYImg2, markerNumImg1, markerNumImg2 = self.findCentroidOfMarkers(img1, img2, full_mask_Img1, full_mask_Img2)

                    # Append data to CSV file
                    if markerNumImg1!=self.CDMMarkerNum or markerNumImg2!=self.CDMMarkerNum:
                        length = max(markerNumImg1,markerNumImg2)
                        if markerNumImg1 > markerNumImg2:
                            imageIndex = 2
                            markerIndex = markerNumImg2
                        if markerNumImg1 < markerNumImg2:
                            imageIndex = 1
                            markerIndex = markerNumImg1
                        if markerNumImg1 == markerNumImg2:
                            imageIndex = 1
                            markerIndex = markerNumImg1

                    else:
                        imageIndex = 0
                        length = markerNumImg1
                    
                    counter +=1
                    for i in range(length):
                        
                        if imageIndex == 0:
                            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                            with open(csv_Cam1and2, 'a', newline='') as csvfile:
                                    csv_writer = csv.writer(csvfile)
                                    csv_writer.writerow([counter, markerNumImg1, sorted_cXImg1[i], sorted_cYImg1[i], counter, markerNumImg2, sorted_cXImg2[i], sorted_cYImg2[i]])

                        else:
                            if i <= markerIndex-1:
                                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                                with open(csv_Cam1and2, 'a', newline='') as csvfile:
                                    csv_writer = csv.writer(csvfile)
                                    csv_writer.writerow([counter, markerNumImg1, sorted_cXImg1[i], sorted_cYImg1[i], counter, markerNumImg2, sorted_cXImg2[i], sorted_cYImg2[i]])
                            else:
                                if imageIndex == 1:
                                    os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                                    with open(csv_Cam1and2, 'a', newline='') as csvfile:
                                        csv_writer = csv.writer(csvfile)
                                        csv_writer.writerow([counter, markerNumImg1, 0, 0, counter, markerNumImg2, sorted_cXImg2[i], sorted_cYImg2[i]])
                                if imageIndex == 2:
                                    os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                                    with open(csv_Cam1and2, 'a', newline='') as csvfile:
                                        csv_writer = csv.writer(csvfile)
                                        csv_writer.writerow([counter, markerNumImg1, sorted_cXImg1[i], sorted_cYImg1[i], counter, markerNumImg2, 0, 0])
                    
                    k = cv2.waitKey(1)

                    if k == 27: #Esc key to stop the program
                        break

                    #cv2.imshow(windowName1,maskImg1)
                    #cv2.imshow(windowName2,maskImg2)
                    #cv2.imshow('image1',img1)
                    #cv2.imshow('image2',img2)
                    #self.findMarkerCentroid(maskImg1,maskImg2)
                    hsvImg1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
                    hsvImg2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
                    maskImg1 = cv2.bitwise_and(hsvImg1, hsvImg1, mask=full_mask_Img1)
                    maskImg2 = cv2.bitwise_and(hsvImg2, hsvImg2, mask=full_mask_Img2)
                    image1 = cv2.hconcat([maskImg1, maskImg2])
                    cv2.imshow(windowName2,image1)

                    image2 = cv2.hconcat([image_1, image_2])
                    cv2.imshow(windowName1,image2)
                    
                
            self.cam1.release()
            self.cam2.release()
            cv2.destroyAllWindows()
            

        elif readImage == True:
            totalNumOfFramesCam1 = []
            totalNumOfFramesCam2 = []

            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
            # Read the CSV file and extract Timestamp values
            with open(csv_numOfFrames_Cam1, 'r') as csvfile:
                csv_reader = csv.reader(csvfile)
                
                # Skip the header row
                next(csv_reader)
                
                # Iterate through rows and extract Timestamp values
                for row in csv_reader:
                    numberOfFrames = row
                    totalNumOfFramesCam1.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
            # Read the CSV file and extract Timestamp values
            with open(csv_numOfFrames_Cam2, 'r') as csvfile:
                csv_reader = csv.reader(csvfile)
                
                # Skip the header row
                next(csv_reader)
                
                # Iterate through rows and extract Timestamp values
                for row in csv_reader:
                    numberOfFrames = row
                    totalNumOfFramesCam2.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
            # CSV file initialization
            with open(csv_Cam1and2, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounterImg1', 'Number of Markers In Img1', 'cXImg1', 'cYImg1', 'FrameCounterImg2', 'Number of Markers In Img2', 'cXImg2', 'cYImg2'])

            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
            # CSV file initialization
            with open(csv_imageCounterMoreMarkerDetected, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounterImg1', 'Number of Markers In Img1', 'FrameCounterImg2', 'Number of Markers In Img2'])
            
            
            if totalNumOfFramesCam1[0] == totalNumOfFramesCam2[0]:
                num = 0
                while num < totalNumOfFramesCam1[0]:

                    #Load Image of Cam1
                    img1 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera1/' + '/image1_' + str(num+1) + '.png')

                    #Load Image of Cam2
                    img2 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera2/' + '/image2_' + str(num+1) + '.png')

                    # if num+1>=2272:
                    #     #Load Image of Cam1
                    #     img1 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera1/' + '/image1_' + str(num+2) + '.png')

                    #     #Load Image of Cam2
                    #     img2 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera2/' + '/image2_' + str(num+2) + '.png')
                    
                    img1, img2 = self.undistortRectify(img1, img2, 'stereoCameraCalibration')
                    
                    hsvImg1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
                    hsvImg2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

                    full_mask_Img1, full_mask_Img2 = self.addHSVFilter(img1, img2)

                    image_1, image_2, sorted_cXImg1, sorted_cYImg1, sorted_cXImg2, sorted_cYImg2, markerNumImg1, markerNumImg2 = self.findCentroidOfMarkers(img1, img2, full_mask_Img1, full_mask_Img2)
                        
                    counter = num + 1

                    print('Counter:',counter)
                    print('Number of marker in image 1:',markerNumImg1)
                    print('Number of marker in image 2:',markerNumImg2)
                    
                    if markerNumImg1!=self.CDMMarkerNum or markerNumImg2!=self.CDMMarkerNum:
                        
                        os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
                        with open(csv_imageCounterMoreMarkerDetected, 'a', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow([counter, markerNumImg1, counter, markerNumImg2])

                        self.setHSVFilter(imageReading = True, liveStreaming = False, WindowName_Image_CentroidOfMarkers = 'Original image whith centroid of markers (Image 1 and 2)', windowCam1 = 'HSV filter of Camera 1', windowCam2 = 'HSV filter of Camera 2', imageFolderName = imageFolderName, counter = counter)
                        
                    else:
                        
                        for i in range(self.CDMMarkerNum):  
                            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
                            with open(csv_Cam1and2, 'a', newline='') as csvfile:
                                    csv_writer = csv.writer(csvfile)
                                    csv_writer.writerow([counter, markerNumImg1, sorted_cXImg1[i], sorted_cYImg1[i], counter, markerNumImg2, sorted_cXImg2[i], sorted_cYImg2[i]])
                        num +=1
            else:
                print('Number of frames for camera1 and camera 2 are not equal!')

    def saveImage_PreciseSynchronization(self, cameraID, created_folder, startTime, stopEvent, syncBarriar, threadLock):
        if cameraID == 0:
            print('here0')
            cam = self.cam1
            #Cam1 Initialization
            global frameCounter_Cam1
            csv_Cam1 = 'Camera1_CounterandTime.csv'
            csv_numOfFrames = 'numOfFramesCam1.csv'
            # Initializing frame counter of Cam1
            frameCounter_Cam1 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam1, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam1', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFrames, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam1'])
        elif cameraID == 1:
            print('here1')
            cam = self.cam2
            #Cam2 Initialization
            global frameCounter_Cam2
            csv_Cam2 = 'Camera2_CounterandTime.csv'
            csv_numOfFrames = 'numOfFramesCam2.csv'
            # Initializing frame counter of Cam2
            frameCounter_Cam2 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam2, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam2', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFrames, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam2'])


        while True:
        
            
            # Wait for all threads to reach the barrier
            #print('while')
            syncBarriar.wait()
            #print('after barrier')
            success, frame = cam.read()
            
            #print('image capture')
            
            syncBarriar.wait()
            currentTime = time.time()

            if not success:
                print("Error reading frame from camera")
                break
            
            timestamp = currentTime - startTime

            if cameraID == 0:
                #print('0')
                # Save image
                with threadLock:
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera1/' + '/image1_' + str(frameCounter_Cam1+1) + '.png', frame)
         
                frameCounter_Cam1 +=1
                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                # Append data to CSV file
                # Acquire the lock before writing to the CSV file
                with threadLock:
                    with open(csv_Cam1, 'a', newline='') as csvfile:
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow([frameCounter_Cam1, timestamp])

            if cameraID == 1:
                #print('1')
                # Save image
                with threadLock:
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera2/' + '/image2_' + str(frameCounter_Cam2+1) + '.png', frame)
                
                frameCounter_Cam2 +=1
                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                # Append data to CSV file
                # Acquire the lock before writing to the CSV file
                with threadLock:
                    with open(csv_Cam2, 'a', newline='') as csvfile:
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow([frameCounter_Cam2, timestamp])
            
            
            if stopEvent.is_set():
                if cameraID == 0:
                    print('breaking1')
                    break
                elif cameraID == 1:
                    print('breaking2')
                    break
        #print('cam1 counter')
        #print(frameCounter_Cam1)
        if cameraID == 0:
            with open(csv_numOfFrames, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam1])
        #print('cam2 counter')
        #print(frameCounter_Cam2)
        if cameraID == 1:
            with open(csv_numOfFrames, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([frameCounter_Cam2])
                
        cam.release()  
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # def costumBarrier_Cam(self, threadCondition, targetThreadNum):
    #     #reachedThreatNum = 0
        
    #     global reachedThreatNum_Cam
    #     with threadCondition:
    #         reachedThreatNum_Cam += 1
    #         #print('reachedThreatNum: ', reachedThreatNum_Cam)
    #         if reachedThreatNum_Cam == targetThreadNum:
    #             threadCondition.notify_all()
    #         else:
    #             while reachedThreatNum_Cam < targetThreadNum:
    #                 #print('reachedThreatNum in while:',reachedThreatNum_Cam)
    #                 threadCondition.wait()
    #                 #itr +=1
    #                 #print('itr:',itr)
    #                 #if itr == 100:
    #                     #print('in 20')
    #                     #threadCondition.notify_all()
    #                     #break
    #             #itr = 0
    #             reachedThreatNum_Cam = 0
    #             #print('out of else')

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

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
        #with threadCondition:
            #reachedThreatNum_StopEvent += 1
            #print('reachedThreatNum_StopEvent: ', reachedThreatNum_StopEvent)
            #if reachedThreatNum_StopEvent == targetThreadNum:
                #threadCondition.notify_all()
            #else:
                #while reachedThreatNum_StopEvent < targetThreadNum:
                    #print('reachedThreatNum_StopEvent in while:',reachedThreatNum_StopEvent)
                    #threadCondition.wait()
                    #itr +=1
                    #print('itr:',itr)
                    #if itr == 100:
                        #print('in 20')
                        #threadCondition.notify_all()
                        #break
                #itr = 0
                #reachedThreatNum_StopEvent = 0
                #print('out of else')


    def saveImage_PreciseSynchronization1(self, cameraID, created_folder, startTime, stopEvent, threadLock, threadCondition, targetThreadNum_Synchronization, targetThreadNum_StopEvent, flag):
        if cameraID == 0:
            cam = self.cam1
            #Cam1 Initialization
            global frameCounter_Cam1
            csv_Cam1 = 'Camera1_CounterandTime.csv'
            csv_numOfFramesCam1 = 'numOfFramesCam1.csv'
            # Initializing frame counter of Cam1
            frameCounter_Cam1 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam1, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam1', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFramesCam1, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam1'])
        elif cameraID == 1:
            cam = self.cam2
            #Cam2 Initialization
            global frameCounter_Cam2
            csv_Cam2 = 'Camera2_CounterandTime.csv'
            csv_numOfFramesCam2 = 'numOfFramesCam2.csv'
            # Initializing frame counter of Cam2
            frameCounter_Cam2 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam2, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam2', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFramesCam2, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam2'])

        
        while True:

        
            
            # Wait for all threads to reach the barrier
            #reachedThreatNum = 0
            #itr = 0
            #print(cameraID,'barrier 1 started')
            if flag.is_set() == False:
                costumBarrier_Synchronization(threadCondition,targetThreadNum_Synchronization)
                success, frame = cam.read()
                #print(cameraID,'pass barrier 1')
                #syncBarriar.wait()
                #print('after barrier')
                
                #print('image capture')
                
                #syncBarriar.wait()

                #print(cameraID,'barrier 2 started')
                #self.costumBarrier(threadCondition,targetThreadNum)
                currentTime = time.time()
                #print(cameraID,'pass barrier 2')


                if not success:
                    print("Error reading frame from camera")
                    break
                #print(cameraID,'Time calculated')
                timestamp = currentTime - startTime
                #print(cameraID,'write image')
                if cameraID == 0:
                    #print('0')
                    # Save image
                    with threadLock:
                        cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera1/' + '/image1_' + str(frameCounter_Cam1+1) + '.png', frame)
            
                    frameCounter_Cam1 +=1
                    os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                    # Append data to CSV file
                    # Acquire the lock before writing to the CSV file
                    with threadLock:
                        with open(csv_Cam1, 'a', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow([frameCounter_Cam1, timestamp])

                if cameraID == 1:
                    #print('1')
                    # Save image
                    with threadLock:
                        cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera2/' + '/image2_' + str(frameCounter_Cam2+1) + '.png', frame)
                    
                    frameCounter_Cam2 +=1
                    os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                    # Append data to CSV file
                    # Acquire the lock before writing to the CSV file
                    with threadLock:
                        with open(csv_Cam2, 'a', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            csv_writer.writerow([frameCounter_Cam2, timestamp])
            
            
            if flag.is_set():
                #print('in flag')
                #time.sleep(5)
                #print('in sop')
                print('Flag of camera ', cameraID, 'is set')
                costumBarrier_StopEvent(threadLock,targetThreadNum_StopEvent, stopEvent)
                print('Thread of camera ', cameraID, 'break')
                break
        #print('cam1 counter')
        #print(frameCounter_Cam1)
        if cameraID == 0:
            with open(csv_numOfFramesCam1, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam1])
        #print('cam2 counter')
        #print(frameCounter_Cam2)
        if cameraID == 1:
            with open(csv_numOfFramesCam2, 'a', newline='') as csvfile: 
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([frameCounter_Cam2])
                
        cam.release()             
    

    def saveImage_PreciseSynchronization_Cam1(self, created_folder, startTime, stopEvent, syncBarriar, threadLock):
            global frameCounter_Cam1
            csv_Cam1 = 'Camera1_CounterandTime.csv'
            csv_numOfFrames = 'numOfFramesCam1.csv'
            # Initializing frame counter of Cam1
            frameCounter_Cam1 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam1, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam1', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFrames, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam1'])

            #previousTime = time.time() #second
        
            while True:
                # Wait for all threads to reach the barrier
                #syncBarriar.wait()
                success, frame = self.cam1.read()
                #syncBarriar.wait()
                currentTime = time.time()

                if not success:
                    print("Error reading frame from camera 1")
                    break
                
                timestamp = currentTime - startTime

                # Save image
                with threadLock:
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera1/' + '/image1_' + str(frameCounter_Cam1+1) + '.png', frame)
                
                frameCounter_Cam1 +=1
                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                # Append data to CSV file
                # Acquire the lock before writing to the CSV file
                with threadLock:
                    with open(csv_Cam1, 'a', newline='') as csvfile:
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow([frameCounter_Cam1, timestamp])
                
                
                print('Thread1')
                
                if stopEvent.is_set():
                    print('break1')
                    break

            with open(csv_numOfFrames, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam1])
            
                
            self.cam1.release()

    def saveImage_PreciseSynchronization_Cam2(self, created_folder, startTime, stopEvent, syncBarriar, threadLock):
            global frameCounter_Cam2
            csv_Cam2 = 'Camera2_CounterandTime.csv'
            csv_numOfFrames = 'numOfFramesCam2.csv'
            # Initializing frame counter of Cam2
            frameCounter_Cam2 = 0

            os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
            # CSV file initialization
            with open(csv_Cam2, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['FrameCounter_Cam2', 'Timestamp (s)'])

            # CSV file initialization
            with open(csv_numOfFrames, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Number of Frames_Cam2'])

            #previousTime = time.time() #second
        
            while True:
                # Wait for all threads to reach the barrier
                #syncBarriar.wait()
                
                success, frame = self.cam2.read()
                #syncBarriar.wait()
                currentTime = time.time()
                
                if not success:
                    print("Error reading frame from camera 2")
                    break
                
                timestamp = currentTime - startTime

                # Save image
                with threadLock:
                    cv2.imwrite(self.directory + '/' + self.folderName + '/' + created_folder + '/' + 'Camera2/' + '/image2_' + str(frameCounter_Cam2+1) + '.png', frame)
                
                frameCounter_Cam2 +=1
                os.chdir(self.directory + '/' + self.folderName + '/' + created_folder)
                # Append data to CSV file
                # Acquire the lock before writing to the CSV file
                with threadLock:
                    with open(csv_Cam2, 'a', newline='') as csvfile:
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow([frameCounter_Cam2, timestamp])
                
                
                print('Thread2')
                if stopEvent.is_set():
                    print('break2')
                    break

            with open(csv_numOfFrames, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([frameCounter_Cam2])
                
            self.cam2.release()            

    def videoDisplayTwoImage(self,imageFolderName):

        csv_Cam1and2 = 'centroidOfMarkers.csv'
        csv_numOfFrames_Cam1 = 'numOfFramesCam1.csv'
        csv_numOfFrames_Cam2 = 'numOfFramesCam2.csv'

        windowName = 'Image 1 and 2'

        totalNumOfFramesCam1 = []
        totalNumOfFramesCam2 = []

        FrameCounterImg1 = []
        numMarkerImg1 = []
        cXImg1 = []
        cYImg1 = []

        FrameCounterImg2 = []
        numMarkerImg2 = []
        cXImg2 = []
        cYImg2 = []


        os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
        # Read the CSV file and extract Timestamp values
        with open(csv_numOfFrames_Cam1, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                numberOfFrames = row
                totalNumOfFramesCam1.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

        os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
        # Read the CSV file and extract Timestamp values
        with open(csv_numOfFrames_Cam2, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            
            # Skip the header row
            next(csv_reader)
            
            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                numberOfFrames = row
                totalNumOfFramesCam2.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

        os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
        # Read the CSV file 
        with open(csv_Cam1and2, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)

            # Skip the header row
            next(csv_reader)

            # Iterate through rows and extract Timestamp values
            for row in csv_reader:
                FrameCounter_Img1, numMarker_Img1, cX_Img1, cY_Img1, FrameCounter_Img2, numMarker_Img2, cX_Img2, cY_Img2= row

                FrameCounterImg1.append(int(FrameCounter_Img1))
                numMarkerImg1.append(int(numMarker_Img1)) 
                cXImg1.append(int(cX_Img1))
                cYImg1.append(int(cY_Img1)) 

                FrameCounterImg2.append(int(FrameCounter_Img2))
                numMarkerImg2.append(int(numMarker_Img2)) 
                cXImg2.append(int(cX_Img2)) 
                cYImg2.append(int(cY_Img2))

        counter = 1
        index = 0
        
        while counter <= totalNumOfFramesCam2[0]:
            
            #Load Image of Cam1
            img1 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera1/' + '/image1_' + str(counter) + '.png')
            
            #Load Image of Cam2
            img2 = cv2.imread(self.directory + '/' + self.folderName + '/' + imageFolderName + '/' + 'Camera2/' + '/image2_' + str(counter) + '.png')

            img1, img2 = self.undistortRectify(img1, img2, 'stereoCameraCalibration')

            for i in range(index,index+self.CDMMarkerNum):

                cv2.circle(img1, (cXImg1[i],cYImg1[i]), 3, (255,255,255), -1)
                cv2.circle(img2, (cXImg2[i],cYImg2[i]), 3, (255,255,255), -1)
            
            image = cv2.hconcat([img1, img2])
            cv2.imshow(windowName,image)
            
            time.sleep(0.01)
            k = cv2.waitKey(1)
            if k == 27: #Esc key to stop the program
                break

            index=i+1
            counter+=1

        cv2.destroyAllWindows()

    
    def stereoCameraCalibration(self, imageFolderName, chessboardNumOfSquareInLength = 9, chessboardNumOfSquareInWidth = 6, size_of_chessboard_squares_mm = 4):
    ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################    
        boolCam1 = 0
        boolCam2 = 0

        chessboardSize = (chessboardNumOfSquareInLength,chessboardNumOfSquareInWidth)

        # Finding frame size
        cam1_images = []
        cam2_images = []
        os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
        for img1_file in glob.glob('Camera1/*.png'):
            img1 = cv2.imread(img1_file)
            cam1_images.append(img1)

        for img2_file in glob.glob('Camera2/*.png'):
            img2 = cv2.imread(img2_file)
            cam2_images.append(img2)

        #cam1_images[*].shape[::] is a tuple (height, width, channels)
        #For calibartion we need (width,height) -->cam1_images[*].shape[1::-1]

        if self.frame_Height == int(cam1_images[0].shape[-3]) and self.frame_Width == int(cam1_images[0].shape[-2]):
            print('width and height of cam1 images are equal to the specified values.')
            boolCam1 = 1
        else:
            print('width and height of cam1 images are NOT equal to the specified values.')

        if self.frame_Height == int(cam2_images[0].shape[-3]) and self.frame_Width == int(cam2_images[0].shape[-2]):
            print('width and height of cam2 images are equal to the specified values.')
            boolCam2 = 1
        else:
            print('width and height of cam2 images are NOT equal to the specified values.')

        if boolCam1 == 1 and boolCam2 == 1:
            print('here')
            frameSize = (self.frame_Width,self.frame_Height) # cam1_images[*].shape[1::-1]
            #frameSize = (1024,768)

            # termination criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
            objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

            #size_of_chessboard_squares_mm = 20
            objp = objp * size_of_chessboard_squares_mm

            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints1 = [] # 2d points in image plane.
            imgpoints2 = [] # 2d points in image plane.

            os.chdir(self.directory + '/' + self.folderName + '/' + imageFolderName)
            images1 = sorted(glob.glob('Camera1/*.png'))
            images2 = sorted(glob.glob('Camera2/*.png'))

            for img_1, img_2 in zip(images1, images2):

                img1 = cv2.imread(img_1)
                img2 = cv2.imread(img_2)
                gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                #gray1 = img1
                #gray2 = img2

                # Find the chess board corners
                ret1, corners1 = cv2.findChessboardCorners(gray1, chessboardSize, None)
                ret2, corners2 = cv2.findChessboardCorners(gray2, chessboardSize, None)
                print('cam1 chessboard corners: ', ret1)
                print('cam2 chessboard corners: ', ret2)

                # If found, add object points, image points (after refining them)
                if ret1 and ret2 == True:
                    
                    objpoints.append(objp)

                    corners1 = cv2.cornerSubPix(gray1, corners1, (11,11), (-1,-1), criteria)
                    imgpoints1.append(corners1)

                    corners2 = cv2.cornerSubPix(gray2, corners2, (11,11), (-1,-1), criteria)
                    imgpoints2.append(corners2)

                    # Draw and display the corners
                    cv2.drawChessboardCorners(img1, chessboardSize, corners1, ret1)
                    cv2.imshow('image 1', img1)
                    cv2.drawChessboardCorners(img2, chessboardSize, corners2, ret2)
                    cv2.imshow('image 2', img2)
                    cv2.waitKey(1000)


            cv2.destroyAllWindows()




        ############## CALIBRATION #######################################################

        ret1, cameraMatrix1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, frameSize, None, None)
        height1, width1, channels1 = img1.shape
        newCameraMatrix1, roi_1 = cv2.getOptimalNewCameraMatrix(cameraMatrix1, dist1, (width1, height1), 1, (width1, height1))

        ret2, cameraMatrix2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints2, frameSize, None, None)
        height2, width2, channels2 = img2.shape
        newCameraMatrix2, roi_2 = cv2.getOptimalNewCameraMatrix(cameraMatrix2, dist2, (width2, height2), 1, (width2, height2))



        ########## Stereo Vision Calibration #############################################

        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
        # Hence intrinsic parameters are the same 

        criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
        retStereo, newCameraMatrix1, dist1, newCameraMatrix2, dist2, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objpoints, imgpoints1, imgpoints2, newCameraMatrix1, dist1, newCameraMatrix2, dist2, gray1.shape[::-1], criteria_stereo, flags)
        print(trans[0],trans[1],trans[2])



        ########## Stereo Rectification #################################################

        rectifyScale= 1
        rect1, rect2, projMatrix1, projMatrix2, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrix1, dist1, newCameraMatrix2, dist2, gray1.shape[::-1], rot, trans, rectifyScale,(0,0))

        stereoMap1 = cv2.initUndistortRectifyMap(newCameraMatrix1, dist1, rect1, projMatrix1, gray1.shape[::-1], cv2.CV_16SC2)
        stereoMap2 = cv2.initUndistortRectifyMap(newCameraMatrix2, dist2, rect2, projMatrix2, gray2.shape[::-1], cv2.CV_16SC2)

        os.chdir(self.directory + '/' + self.folderName )
        print("Saving parameters!")
        cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

        cv_file.write('stereoMap1_x',stereoMap1[0])
        cv_file.write('stereoMap1_y',stereoMap1[1])
        cv_file.write('stereoMap2_x',stereoMap2[0])
        cv_file.write('stereoMap2_y',stereoMap2[1])

        cv_file.write('projectionMatrix1', projMatrix1)
        cv_file.write('projectionMatrix2', projMatrix2)

        cv_file.release()

    def undistortRectify(self, frameCam1, frameCam2, folder_Name_Stereo_Matrices):

        # Camera parameters to undistort and rectify images
        os.chdir(self.directory + '/' + folder_Name_Stereo_Matrices )
        cv_file = cv2.FileStorage()
        cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

        stereoMap1_x = cv_file.getNode('stereoMap1_x').mat()
        stereoMap1_y = cv_file.getNode('stereoMap1_y').mat()
        stereoMap2_x = cv_file.getNode('stereoMap2_x').mat()
        stereoMap2_y = cv_file.getNode('stereoMap2_y').mat()


        # Undistort and rectify images
        undistorted1= cv2.remap(frameCam1, stereoMap1_x, stereoMap1_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        undistorted2= cv2.remap(frameCam2, stereoMap2_x, stereoMap2_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)


        return undistorted1, undistorted2 
    
    
    def markerTriangulation(self, markerPositionCam1, markerPositionCam2, folder_Name_Stereo_Matrices):
        # markerPositionCam1 is 2xN array extracted from undistorted frame (N is the number of measurements)
        # markerPositionCam2 is 2xN array extracted from undistorted frame (N is the number of measurements)

        # Camera parameters to undistort and rectify images
        os.chdir(self.directory + '/' + folder_Name_Stereo_Matrices )
        cv_file = cv2.FileStorage()
        cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

        projectionMatrix1 = cv_file.getNode('projectionMatrix1').mat() #3x4 matrix
        projectionMatrix2 = cv_file.getNode('projectionMatrix2').mat() #3x4 matrix

        numRowsproj1 = len(projectionMatrix1)
        numColsproj1 = len(projectionMatrix1[0])

        numRowsproj2 = len(projectionMatrix2)
        numColsproj2 = len(projectionMatrix2[0])

        if numRowsproj1!=3 or numRowsproj2!=3:
            print('Number of the row in projection matrix should be 3')

        if numColsproj1!=4 or numColsproj2!=4:
            print('Number of the row in projection matrix should be 3')


        numRowsCam1 = len(markerPositionCam1)
        numColsCam1 = len(markerPositionCam1[0])

        numRowsCam2 = len(markerPositionCam2)
        numColsCam2 = len(markerPositionCam2[0])
        
        if numRowsCam1 == 2 and numRowsCam2 == 2:
            if numColsCam1 == numColsCam2:
                #points4D is a 4xN (OR 3xN [x y scale]) array of reconstructed points in homogeneous coordinates. [x y z scale]
                points4D = cv2.triangulatePoints(projectionMatrix1, projectionMatrix2, markerPositionCam1, markerPositionCam2)
                
                points3D = cv2.convertPointsFromHomogeneous(points4D.T) # point3D is Nx3 [x y z]
            else:
                print('Number of measurements for camera 1 and camera 2 are not same')

        else:
            print('Marker positions should be a 2xN array.')

        return points3D
        

    
    
        


        