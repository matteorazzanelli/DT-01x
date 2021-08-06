#!/usr/bin/env python
# coding: utf-8

# In[40]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    # steer_matrix_left_lane = -np.eye(shape[0],shape[1])*0.1
    # steer_matrix_left_lane = -np.block([
    #     [np.zeros((240,320), dtype=np.ndarray), np.zeros((240,320), dtype=np.ndarray)],
    #     [np.eye(240, dtype=np.ndarray), np.zeros((240,320+80), dtype=np.ndarray)]*0.1
    # ])
    steer_matrix_left_lane = -np.ones((1,shape[1]), dtype=np.ndarray)*(1/shape[1])
    # print(shape)

    return steer_matrix_left_lane


# In[41]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    # steer_matrix_right_lane = np.eye(shape[0],shape[1])*0.1
    # steer_matrix_right_lane = np.block([
    #     [np.zeros((240,320), dtype=np.ndarray), np.zeros((240,320), dtype=np.ndarray)],
    #     [np.zeros((240,320+80), dtype=np.ndarray), np.eye(240, dtype=np.ndarray)]
    # ])*0.1
    steer_matrix_right_lane = np.ones((1,shape[1]), dtype=np.ndarray)*(1/shape[1])
    # print(shape)

    return steer_matrix_right_lane


# In[37]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np
import sys


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    # RGB, HSV, GRAY
    imgrgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    h, w, _ = image.shape
    
    # BLUR
    sigma = 3
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    
    # SOBEL
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    # MAGNITUDE FILTER
    threshold = 60 # CHANGE ME
    mask_mag = (Gmag > threshold)
    mask_mag = mask_mag*Gmag
    
    # COLOR FILTER
    sensitivity = 100
    white_lower_hsv = np.array([0, 0, 255-sensitivity])         # CHANGE ME
    white_upper_hsv = np.array([179, sensitivity, 255])   # CHANGE ME
    yellow_lower_hsv = np.array([19, 100, 100])
    yellow_upper_hsv = np.array([28, 255, 255])
    #white_lower_hsv = np.array([0,0,168])
    #white_upper_hsv = np.array([172,111,255])
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    # ORIENTATION
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_neg = (sobely < 0)
    
    # SUM-UP
    mask_left_edge = mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    # fig = plt.figure(figsize = (10,5))
    # ax1 = fig.add_subplot(1,2,1)
    # ax1.hist(np.extract(mask_left_edge, Gdir).flatten(), bins=30)
    # ax1.set_title('Gradient Direction Histogram (Left Edge)')
    # ax2 = fig.add_subplot(1,2,2)
    # ax2.hist(np.extract(mask_right_edge, Gdir).flatten(), bins=30)
    # ax2.set_title('Gradient Direction Histogram (Right Edge)');
    
    # mask_white2 = cv2.inRange(imghsv2, white_lower_hsv, white_upper_hsv)
    # mask_yellow2 = cv2.inRange(imghsv2, yellow_lower_hsv, yellow_upper_hsv)
    # fig2 = plt.figure(figsize = (20,10))
    # ax5 = fig2.add_subplot(1,3,3)
    # ax5.imshow(imgrgb2)
    # ax5.imshow(mask_white2, cmap='jet', alpha=0.5)
    # ax5.set_title('Filtered by Color (White)'), ax5.set_xticks([]), ax5.set_yticks([]);
    # ax6 = fig2.add_subplot(1,3,2)
    # ax6.imshow(imgrgb2)
    # ax6.imshow(mask_yellow2, cmap='jet', alpha=0.5)
    # ax6.set_title('Filtered by Color (Yellow)'), ax6.set_xticks([]), ax6.set_yticks([]);
    
    # print((nonzero(mask_left_edge)))
    
    return (mask_left_edge, mask_right_edge)

