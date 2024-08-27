import cv2
from stereoVisionClass import stereoVision
import scipy.io
import os

#Specify the directory for saving the data
target_directory = "/home/golchehr/bigss/StereoVision/Python"
mode = 'stereoCameraCalibration' #Specify the folder name where the image is saved
imageFolderName = '2024-03-13_16-27-40' #sub folder name

# I use Matlab stereo camera calibration tool box to find stereo params.
# we only need to extract intrinsic matrices of camera 1 and camera 2 and distortion coefficient of them from stereo params

# If liveStreaming = True, mode and imageFolderName are not involved


# Instantiate the stereoCameraControl Class
stereoCamera = stereoVision(Cam1ID=cv2.CAP_FIREWIRE+0, Cam2ID=cv2.CAP_FIREWIRE+1, setTrackbar='False', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = target_directory, mode = mode, CDM_MarkerNum = 30, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties')
stereoCamera.stereoCameraCalibration(imageFolderName, chessboardNumOfSquareInLength = 8, chessboardNumOfSquareInWidth = 5, size_of_chessboard_squares_mm = 4)
