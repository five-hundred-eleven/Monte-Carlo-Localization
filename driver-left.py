#!/usr/bin/env python
import sys, os
import roslib; roslib.load_manifest('uml_hmm')
import rospy
from rospy import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from math import *
from sensing_model_and_map import * 

import pygame

''' 
    Map notes:
    Coordinate system of map goes from -30.00 to 30.00.
    First door:     -20 to -18      confidence[100] to confidence[120]
    Second door:    -12.5 to 10.5   confidence[175] to confidence[195]  
    Third door:     10 to 12        confidence[400] to confidence[420] 
'''

velocity = -4
confidence = np.array([1.0/600.0]*600)	
numBins = 600
height = 200
scaleFactor = height*8

''' 
pygame setup
'''
pygame.init()
screen = pygame.display.set_mode((numBins, height))
font = pygame.font.SysFont("Aeriel", 30)


''' precompute the gaussian curve for sigma = e
    length of the curve is dependant on velocity '''
gaussian_len = -velocity*2 + 2
gaussian = [gauss(i, velocity, e) for i in range(gaussian_len)]

''' doorKnocker is the callback function for the door sensor subscriber '''
def doorKnocker(data):
    global confidence
    ''' Move and spread the existing confidence '''	
    for i in range(600):
        temp = confidence[i]
        confidence[i] = 0
        for j in range(i, i - gaussian_len if i - gaussian_len >= 0 else 0, -1):
            confidence[j] += temp*gaussian[j - i]
    ''' Measurement step: adjust confidence according to the sensor input '''
    if data.data == 'Door':
        eta = 1.0/sum([p_door(i/10.0)*confidence[i] for i in range(600)])
        for i in range(600):
            confidence[i] = eta * confidence[i] * p_door(i/10.0)
    else:
        eta = 1.0/sum([p_wall(i/10.0)*confidence[i] for i in range(600)])
        for i in range(600):
            confidence[i] = eta * confidence[i] * p_wall(i/10.0)
    ''' Normalization '''
    confidenceSum = sum(confidence)
    if confidenceSum != 1.0:
        confidence = map(lambda x: x/confidenceSum, confidence)

def draw(background = None):
    global screen, font, numBins, height

    screen.fill((0, 0, 0))
    if background is not None:
        screen.blit(background, (0, 0))
        
    '''
    get confidence bins and scale
    '''
    vec = map(lambda x: x * scaleFactor, confidence)
    ''' 
    draw the bins
    '''
    for i, v in enumerate(vec):
        pygame.draw.line(screen, (255, 0, 0), (i, height - v), (i, height))

    pygame.display.flip()

def get_background_img(filename):
    fullname = os.path.join(filename)
    try:
        img = pygame.image.load(fullname)
    except pygame.error, message:
        return None
    img = img.convert()
    return img

if __name__ == '__main__':
    rospy.init_node('robot_driver')
    controller = rospy.Publisher('robot/cmd_vel', Twist)
    issue_command = controller.publish

    cmd = Twist()
    cmd.linear.x = velocity 

    bg = get_background_img('../share/hmm.png')

    rospy.Subscriber('robot/wall_door_sensor', String, doorKnocker)
    for i in range(290):
        issue_command(cmd)
        draw(background = bg)
        sleep(0.1)
