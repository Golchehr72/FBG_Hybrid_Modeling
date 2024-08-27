import cv2
from stereoVisionClass import stereoVision

#Specify the directory for saving the data
#target_directory = "/home/golchehr/bigss/StereoVision/Python"
target_directory = "/home/golchehr/bigss/catkin_ws/src/fbg_hybrid_modeling"
mode = 'cameraLiveStreaming' #Specify the folder name where the image is saved
imageFolderName = '2024-02-05_15-20-27' #sub folder name

mode = 'stereoCameraAccuracyMeasurment'
imageFolderName = '2024-02-09_15-11-56'

# If liveStreaming = True, mode and imageFolderName are not involved


# Instantiate the stereoCameraControl Class
stereoCamera = stereoVision(Cam1ID=cv2.CAP_FIREWIRE+1, Cam2ID=cv2.CAP_FIREWIRE+0, setTrackbar='False', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = target_directory, mode = mode, CDM_MarkerNum = 30, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties')
stereoCamera.setHSVFilter(imageReading = False, liveStreaming = True, WindowName_Image_CentroidOfMarkers = 'Original image whith centroid of markers (Image 1 and 2)', windowCam1 = 'HSV filter of Camera 1', windowCam2 = 'HSV filter of Camera 2', imageFolderName = imageFolderName, counter = 4)

