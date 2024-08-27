import cv2
from stereoVisionClass import stereoVision
import os
import numpy as np
import csv
import glob
from numpy.linalg import norm
import math
import statistics

totalNumOfMarker = 30

#Specify the directory for saving the data
target_directory = "/home/golchehr/bigss/catkin_ws/src/fbg_hybrid_modeling"
mode = 'experiment'
folder_Name_Stereo_Matrices = 'stereoCameraCalibration'
csv_CentroidOfMarkersAfterTriangulation = 'CentroidOfMarkersAfterTriangulation.csv'
csv_edge_of_triangle = 'triangleEdgesAfterTriangulation.csv'

# Instantiate the stereoCameraControl Class
stereoCamera = stereoVision(Cam1ID=cv2.CAP_FIREWIRE+1, Cam2ID=cv2.CAP_FIREWIRE+0, setTrackbar='False', IsoSpeed=800, frameWidth=1024, frameHeight=768, target_directory = target_directory, mode = mode, CDM_MarkerNum = totalNumOfMarker, folderName_HSV_Value = 'HSV_Value', folderName_videoProperties = 'video_Capture_Properties')

#~~~~~~~~~~~~~~~~~Centroid of marker extraction~~~~~~~~~~~~~~~~~
#imageFolderName = '2024-02-09_15-11-56'
experimentFolderName = []
os.chdir(target_directory + '/' + mode )
folderIndex = 0
for file in glob.glob('*'):
    folderIndex += 1
    experimentFolderName.append(file)
    print('folderIndex[',folderIndex,']:',file)

folderIndex = input('Enter the folder index: (ex: 1, 2, ...) ')
imageFolderName = experimentFolderName[int(folderIndex)-1]

# If liveStreaming = True, mode and imageFolderName are not involved
stereoCamera.centroidOfCDMMarkerExtraction_AllFrames(liveStreaming = False, readImage = True, imageFolderName = imageFolderName)


#~~~~~~~~~~~~~~~~~Displaying centroid of markers of all frame~~~~~~~~~~~~~~~~~
#stereoCamera.videoDisplayTwoImage( imageFolderName = imageFolderName)


#~~~~~~~~~~~~~~~~~Storing Centroid of marker in 2xN array with respect to the camera frame~~~~~~~~~~~~~~~~~
csv_Cam1and2 = 'centroidOfMarkers.csv'
csv_numOfFrames_Cam1 = 'numOfFramesCam1.csv'
csv_numOfFrames_Cam2 = 'numOfFramesCam2.csv'

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

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# CSV file initialization
with open(csv_CentroidOfMarkersAfterTriangulation, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['FrameCounter', 'cX (mm)', 'cY (mm)', 'cZ (mm)'])   


os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Read the CSV file and extract Timestamp values
with open(csv_numOfFrames_Cam1, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    
    # Skip the header row
    next(csv_reader)
    
    # Iterate through rows and extract Timestamp values
    for row in csv_reader:
        numberOfFrames = row
        totalNumOfFramesCam1.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Read the CSV file and extract Timestamp values
with open(csv_numOfFrames_Cam2, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    
    # Skip the header row
    next(csv_reader)
    
    # Iterate through rows and extract Timestamp values
    for row in csv_reader:
        numberOfFrames = row
        totalNumOfFramesCam2.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
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

counter = 0
index = 0
while counter < totalNumOfFramesCam2[0]:
    counter+=1
    markerPositionCam1 = np.zeros((2,totalNumOfMarker))
    markerPositionCam2 = np.zeros((2,totalNumOfMarker))
    colNum = -1
    for i in range(index,index+totalNumOfMarker):
        colNum += 1
        
        markerPositionCam1[0,colNum] = cXImg1[i]
        markerPositionCam1[1,colNum] = cYImg1[i]

        markerPositionCam2[0,colNum] = cXImg2[i]
        markerPositionCam2[1,colNum] = cYImg2[i]

    points3D = stereoCamera.markerTriangulation(markerPositionCam1, markerPositionCam2, folder_Name_Stereo_Matrices)
    
    for ii in range(totalNumOfMarker):
        os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
        with open(csv_CentroidOfMarkersAfterTriangulation, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([counter, points3D[ii][0,0], points3D[ii][0,1], points3D[ii][0,2]])


    k = cv2.waitKey(1)
    if k == 27: #Esc key to stop the program
        break

    index=i+1




#~~~~~~~~~~~~~~~~~Storing Centroid of marker in 2xN array with respect to the CDM base coordinate frame at its proximal end~~~~~~~~~~~~~~~~~
csv_Cam1and2 = 'CentroidOfMarkersAfterTriangulation.csv'
csv_numOfFrames_Cam1 = 'numOfFramesCam1.csv'
csv_numOfFrames_Cam2 = 'numOfFramesCam2.csv'
csv_CentroidOfMarkersInBaseCoordinate = 'CentroidOfMarkersInBaseCoordinate.csv'

totalNumOfFramesCam1 = []
totalNumOfFramesCam2 = []

FrameCounter = []
cX = []
cY = []
cZ = []

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# CSV file initialization
with open(csv_CentroidOfMarkersInBaseCoordinate, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['FrameCounter', 'cX (mm)', 'cY (mm)', 'cZ (mm)'])  

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Read the CSV file and extract Timestamp values
with open(csv_numOfFrames_Cam1, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    
    # Skip the header row
    next(csv_reader)
    
    # Iterate through rows and extract Timestamp values
    for row in csv_reader:
        numberOfFrames = row
        totalNumOfFramesCam1.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Read the CSV file and extract Timestamp values
with open(csv_numOfFrames_Cam2, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    
    # Skip the header row
    next(csv_reader)
    
    # Iterate through rows and extract Timestamp values
    for row in csv_reader:
        numberOfFrames = row
        totalNumOfFramesCam2.append(int(numberOfFrames[0]))  # Assuming numberOfFrames values are stored as strings

os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Read the CSV file 
with open(csv_Cam1and2, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)

    # Skip the header row
    next(csv_reader)

    # Iterate through rows and extract Timestamp values
    for row in csv_reader:
        Frame_Counter, c_X, c_Y, c_Z= row

        FrameCounter.append(int(Frame_Counter)) 
        cX.append(float(c_X))
        cY.append(float(c_Y)) 
        cZ.append(float(c_Z))


counter = 0
index = 0
while counter < totalNumOfFramesCam2[0]:
    counter+=1
    cX_wth_BaseCoordinate = []
    cY_wth_BaseCoordinate = []
    cZ_wth_BaseCoordinate = []
    baseMarkerIndex = index
    for i in range(index,index+totalNumOfMarker):
        
        cX_wth_BaseCoordinate.append(cX[i] - cX[baseMarkerIndex])
        cY_wth_BaseCoordinate.append(cY[i] - cY[baseMarkerIndex])
        cZ_wth_BaseCoordinate.append(cZ[i] - cZ[baseMarkerIndex])
    
    for ii in range(totalNumOfMarker):
        os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
        with open(csv_CentroidOfMarkersInBaseCoordinate, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([counter, cX_wth_BaseCoordinate[ii], cY_wth_BaseCoordinate[ii], cZ_wth_BaseCoordinate[ii]])


    k = cv2.waitKey(1)
    if k == 27: #Esc key to stop the program
        break

    index=i+1





# os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# Frame_Counter = []
# c_X = []
# c_Y = []
# c_Z = []
# # Read the CSV file 
# with open(csv_CentroidOfMarkersAfterTriangulation, 'r') as csvfile:
#     csv_reader = csv.reader(csvfile)

#     # Skip the header row
#     next(csv_reader)

#     # Iterate through rows and extract Timestamp values
#     for row in csv_reader:
        
#         FrameCounter, cX, cY, cZ = row

#         Frame_Counter.append(int(FrameCounter))
#         c_X.append(float(cX)) 
#         c_Y.append(float(cY))
#         c_Z.append(float(cZ))



# os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
# # CSV file initialization
# with open(csv_edge_of_triangle, 'w', newline='') as csvfile:
#     csv_writer = csv.writer(csvfile)
#     csv_writer.writerow(['FrameCounter', 'L1 (mm)', 'L2 (mm)', 'L3 (mm)', 'Error_L1 (mm)', 'Error_L2 (mm)', 'Error_L3 (mm)'])   


# counter = 1
# index = 0
# totalError = []
# while counter <= totalNumOfFramesCam2[0]:
#     vecCX = []
#     vecCY = []
#     for i in range(index,index+totalNumOfMarker):

#         vecCX.append(c_X[i])
#         vecCY.append(c_Y[i])

#     length = []
#     for j in range(0,totalNumOfMarker-1):
#         for k in range(j+1,totalNumOfMarker):
#             X = vecCX[j]-vecCX[k]
#             Y = vecCY[j]-vecCY[k]
#             length.append(norm([X, Y]))

#     sorted_Measured_Length = []       
#     sortedIndices_Measured_Length = np.argsort(length)
#     for ii in sortedIndices_Measured_Length:
#         sorted_Measured_Length.append(length[ii])

#     #Error calculation
#     e = []
#     for ii in range(len(sorted_Measured_Length)):
#         e.append(abs(sorted_Measured_Length[ii] - sorted_Actual_Length[ii]))
#         totalError.append(abs(sorted_Measured_Length[ii] - sorted_Actual_Length[ii]))

#     os.chdir(target_directory + '/' + mode + '/' + imageFolderName)
#     with open(csv_edge_of_triangle, 'a', newline='') as csvfile:
#             csv_writer = csv.writer(csvfile)
#             csv_writer.writerow([counter, sorted_Measured_Length[0], sorted_Measured_Length[1], sorted_Measured_Length[2],e[0], e[1], e[2]])    
        

#     index=i+1
#     counter+=1

# mean_totalError = statistics.mean(totalError)
# std_totalError = statistics.stdev(totalError)
# print(totalError)
# print('Mean error is:',mean_totalError)
# print('Standard deviation of error is:',std_totalError)
    

cv2.destroyAllWindows()
