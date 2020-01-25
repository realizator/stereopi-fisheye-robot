# Copyright (C) 2019 Eugene a.k.a. Realizator, stereopi.com, virt2real team
#
# This file is part of StereoPi tutorial scripts.
#
# StereoPi tutorial is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# StereoPi tutorial is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with StereoPi tutorial.  
# If not, see <http://www.gnu.org/licenses/>.
#
#          <><><> SPECIAL THANKS: <><><>
#
# Thanks to Adrian and http://pyimagesearch.com, as a lot of
# code in this tutorial was taken from his lessons.
#  
# Thanks to RPi-tankbot project: https://github.com/Kheiden/RPi-tankbot
#
# Thanks to rakali project: https://github.com/sthysel/rakali


from picamera import PiCamera
import time
import cv2
import numpy as np
import json
from datetime import datetime

print ("You can press 'Q' to quit this script!")
time.sleep (5)

# Visualization settings
showDisparity = True
showUndistortedImages = True
showColorizedDistanceLine = True

# Depth map default preset
SWS = 5
PFS = 5
PFC = 29
MDS = -30
NOD = 160
TTH = 100
UR = 10
SR = 14
SPWS = 100

# Camera settimgs
cam_width = 1280
cam_height = 480

# Final image capture settings
scale_ratio = 0.5

# Camera resolution height must be dividable by 16, and width by 32
cam_width = int((cam_width+31)/32)*32
cam_height = int((cam_height+15)/16)*16
print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

# Depth Map colors autotune
autotune_min = 10000000
autotune_max = -10000000

# 3D points settings
focal_length = 165.0 #taken from K matrix
tx = 65 #taken from K matrix after calibration
q = np.array([
    [1, 0, 0, -img_width/2],
    [0, 1, 0, -img_height/2],
    [0, 0, 0, focal_length],
    [0, 0, -1/tx,0]
    ])


# Initialize the camera
camera = PiCamera(stereo_mode='side-by-side',stereo_decimate=False)
camera.resolution=(cam_width, cam_height)
camera.framerate = 20
#camera.hflip = True

# Initialize interface windows
cv2.namedWindow("Image")
cv2.moveWindow("Image", 50,100)
cv2.namedWindow("left")
cv2.moveWindow("left", 450,100)
cv2.namedWindow("right")
cv2.moveWindow("right", 850,100)


disparity = np.zeros((img_width, img_height), np.uint8)
sbm = cv2.StereoBM_create(numDisparities=0, blockSize=21)

def stereo_depth_map(rectified_pair):
    dmLeft = rectified_pair[0]
    dmRight = rectified_pair[1]
    disparity = sbm.compute(dmLeft, dmRight)
    local_max = disparity.max()
    local_min = disparity.min()
    #print(local_max, local_min)
    disparity_grayscale = (disparity-autotune_min)*(65535.0/(autotune_max-autotune_min))
    disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0/65535.0))
    disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
    if (showDisparity):
        cv2.imshow("Image", disparity_color)
        key = cv2.waitKey(1) & 0xFF   
        if key == ord("q"):
            quit();
    return disparity_color, disparity_fixtype, disparity

def load_map_settings( fName ):
    global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
    print('Loading parameters from file...')
    f=open(fName, 'r')
    data = json.load(f)
    SWS=data['SADWindowSize']
    PFS=data['preFilterSize']
    PFC=data['preFilterCap']
    MDS=data['minDisparity']
    NOD=data['numberOfDisparities']
    TTH=data['textureThreshold']
    UR=data['uniquenessRatio']
    SR=data['speckleRange']
    SPWS=data['speckleWindowSize']    
    #sbm.setSADWindowSize(SWS)
    sbm.setPreFilterType(1)
    sbm.setPreFilterSize(PFS)
    sbm.setPreFilterCap(PFC)
    sbm.setMinDisparity(MDS)
    sbm.setNumDisparities(NOD)
    sbm.setTextureThreshold(TTH)
    sbm.setUniquenessRatio(UR)
    sbm.setSpeckleRange(SR)
    sbm.setSpeckleWindowSize(SPWS)
    f.close()
    print ('Depth map settings has been loaded from the file '+fName)

# Loading depth map settings
load_map_settings ("3dmap_set.txt")

# Loading stereoscopic calibration data
try:
    npzfile = np.load('./calibration_data/{}p/stereo_camera_calibration.npz'.format(img_height))
except:
    print("Camera calibration data not found in cache, file ", './calibration_data/{}p/stereo_camera_calibration.npz'.format(img_height))
    exit(0)
    
imageSize = tuple(npzfile['imageSize'])
leftMapX = npzfile['leftMapX']
leftMapY = npzfile['leftMapY']
rightMapX = npzfile['rightMapX']
rightMapY = npzfile['rightMapY']
QQ = npzfile['dispartityToDepthMap']

map_width = 320
map_height = 240

min_y = 10000
max_y = -10000
min_x =  10000
max_x = -10000
# Capture the frames from the camera
for frame in camera.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
    t1 = datetime.now()
    pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
    imgLeft = pair_img [0:img_height,0:int(img_width/2)] #Y+H and X+W
    imgRight = pair_img [0:img_height,int(img_width/2):img_width] #Y+H and X+W
    imgL = cv2.remap(imgLeft, leftMapX, leftMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    imgR = cv2.remap(imgRight, rightMapX, rightMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    
    # Taking a strip from our image for lidar-like mode (and saving CPU) 
    imgRcut = imgR [80:160,0:int(img_width/2)]
    imgLcut = imgL [80:160,0:int(img_width/2)]
    rectified_pair = (imgLcut, imgRcut)
    
    # Disparity map calculation
    disparity, disparity_bw, native_disparity  = stereo_depth_map(rectified_pair)

    maximized_line = native_disparity
    
    maxInColumns = np.amax(maximized_line,0)
    points = cv2.reprojectImageTo3D(maxInColumns, QQ)
    xy_projection = np.zeros((map_height , map_width, 1), dtype=np.uint8)

    # "Jumping colors" protection for depth map visualization
    if autotune_max < np.amax(maximized_line):
        autotune_max = np.amax(maximized_line)
    if autotune_min > np.amin(maximized_line):
        autotune_min = np.amin(maximized_line)    
    
    # Choose "closest" points in each column
    maximized_line[0:80,] = maxInColumns
    
    # Colorizing final line
    max_line_tune = (maximized_line-autotune_min)*(65535.0/(autotune_max-autotune_min))
    max_line_gray = cv2.convertScaleAbs(max_line_tune, alpha=(255.0/65535.0))

    # Put all points to the 2D map
    
    # Change map_zoom to adjust visible range!
    map_zoom_y = int(map_height/(max_y-min_y))
    map_zoom_x = int(map_height/(max_x-min_x)) 
    for n, points in enumerate(points):
        cur_y = -points[0][0]
        cur_x = points[0][1]
        max_y = max(cur_y, max_y)
        min_y = min(cur_y, min_y)
        max_x = max(cur_x, max_x)
        min_x = min(cur_x, min_x)
        xx = int(cur_x*map_zoom_x) + int(map_width/2)         # zero point is in the middle of the map
        yy = map_height - int((cur_y-min_y)*map_zoom_y)       # zero point is at the bottom of the map

        # If the point fits on our 2D map - let's draw it!
        if (xx < map_width) and (xx >= 0) and (yy < map_height) and (yy >= 0):
            xy_projection[yy, xx] = max_line_gray[0,n]
    
    #print ("min_y = " + str(min_y) + " max_y = " + str(max_y) + " zoom_x = " + str(map_zoom_x) + " zoom_y = " + str(map_zoom_y))
    xy_projection_color = cv2.applyColorMap(xy_projection, cv2.COLORMAP_JET)
    max_line_color = cv2.applyColorMap(max_line_gray, cv2.COLORMAP_JET)

    # show the frame
    #print ("Autotune: min =", autotune_min, " max =", autotune_max)
    if (showUndistortedImages):
        cv2.imshow("left", imgLcut)
        cv2.imshow("right", imgRcut)    
    if (showColorizedDistanceLine):
        cv2.imshow("Max distance line", max_line_color)
    cv2.imshow("XY projection", xy_projection_color)     
    t2 = datetime.now()
    #print ("DM build time: " + str(t2-t1))


