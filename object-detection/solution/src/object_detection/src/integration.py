#!/usr/bin/env python
# coding: utf-8

# In[6]:


def DT_TOKEN():
    # todo change this to your duckietown token
    dt_token = "dt1-3nT8KSoxVh4Mf6zSKbDRadA1ddindhNuDdmseumFFQ6HDzt-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfSkxSD9S26tXPc4fN1WYM73ZNuALp7QMiW"
    return dt_token

def MODEL_NAME():
    # todo change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5"
    return "yolov5"


# In[7]:


def NUMBER_FRAMES_SKIPPED():
    # todo change this number to drop more frames
    # (must be a positive integer)
    return 3


# In[8]:


# `class` is the class of a prediction
def filter_by_classes(clas):
    # Right now, this returns True for every object's class
    # Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    print(f'class {clas}')
    return True


# In[9]:


# `scor` is the confidence score of a prediction
def filter_by_scores(scor):
    # Right now, this returns True for every object's confidence
    # Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    print(f'scor {scor}')
    return True


# In[10]:


# `bbox` is the bounding box of a prediction, in xyxy format
# So it is of the shape (leftmost x pixel, topmost y pixel, rightmost x pixel, bottommost y pixel)
def filter_by_bboxes(bbox):
    # Like in the other cases, return False if the bbox should not be considered.
    # size = ((bbox[0]-bbox[2])**2+(bbox[1]-bbox[3])**2)**0.5
    # if size < 10:
    #     return False
    # else :
    print(f'bbox {bbox[0]} x {bbox[1]} x {bbox[2]} x {bbox[3]}')
    return True

