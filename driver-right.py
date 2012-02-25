#!/usr/bin/env python
import sys, os
import roslib; roslib.load_manifest('uml_hmm')
import rospy
from rospy import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from uml_hmm.msg import Bins

from numpy import *
from math import *
from sensing_model_and_map import * 

import pygame 

''' 
    Map notes:
    Coordinate system of map goes from -30.00 to 30.00.
    First door:     -20 to -18      bins[100] to bins[120]
    Second door:    -12.5 to 10.5   bins[175] to bins[195]  
    Third door:     10 to 12        bins[400] to bins[420] 
'''

velocity = 4
numBins = 600
maxBin = numBins - 1
height = 200
scaleFactor = height*8

''' 
pygame setup
'''
pygame.init()
screen = pygame.display.set_mode((numBins, height))
font = pygame.font.SysFont("Aeriel", 30)

confidence = [1.0/numBins for i in range(numBins)]

''' precompute the gaussian curve for sigma = e
    length of the curve is dependant on velocity '''
gaussian_len = velocity*2 + 2
gaussian = [gauss(i, velocity, e) for i in range(gaussian_len)]

''' doorKnocker is the callback function for the door sensor subscriber '''
def doorKnocker(data):
    global confidence
    ''' Move and spread the existing confidence '''	
    for i in range(maxBin, -1, -1):
        temp = confidence[i]
        confidence[i] = 0
        for j in range(i, i + gaussian_len if i + gaussian_len < numBins else maxBin):
            confidence[j] += temp*gaussian[j - i]
    ''' Measurement step: adjust confidence according to the sensor inumpyut '''
    if data.data == 'Door':
        eta = 1.0/sum([p_door(i/10.0)*confidence[i] for i in range(numBins)])
        for i in range(numBins):
            confidence[i] = eta * confidence[i] * p_door(i/10.0)
    else:
        eta = 1.0/sum([p_wall(i/10.0)*confidence[i] for i in range(numBins)])
        for i in range(numBins):
            confidence[i] = eta * confidence[i] * p_wall(i/10.0)

def draw(background = None):
    global screen, font, numBins, height

    screen.fill((0, 0, 0))
    if background is not None:
        screen.blit(background, (0, 0))
        
    '''
    get belief bins and scale
    '''
    vec = map(lambda x: x * scaleFactor, confidence)
    ''' 
    draw the bins
    '''
    for i, v in enumerate(vec):
        pygame.draw.line(screen, (255, 0, 0), (i, height - v), (i, height))

    pygame.display.flip()

''' 
Filename is the path to your world map image file, for ps5/6
it should be {ros_workspace}/uml_hmm/share/hmm.png
'''
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

    rospy.Subscriber('robot/wall_door_sensor', String, doorKnocker)

    background_img = get_background_img('../share/hmm.png')

    for i in range(290):
        issue_command(cmd)
        draw(background_img)
        sleep(0.1)
