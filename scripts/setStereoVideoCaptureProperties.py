import cv2
from stereoVisionClass import stereoVision

#Specify the directory for saving the data
target_directory = "/home/golchehr/bigss/StereoVision/Python"
mode = 'videoProperties'

# Instantiate the stereoCameraControl Class
stereoCamera = stereoVision(Cam1ID=cv2.CAP_FIREWIRE+0, Cam2ID=cv2.CAP_FIREWIRE+1, setTrackbar='True', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = target_directory, mode = mode, CDM_MarkerNum = 30, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties')
stereoCamera.trackbarVisualization('Img1','Img2')



