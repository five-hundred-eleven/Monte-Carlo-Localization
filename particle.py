#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('uml_hmm')
import rospy
from rospy import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sensing_model_and_map as smm

import threading
import random
import numpy as np
import pygame
from math import *
from collections import namedtuple

# get direction
if 'left' in [arg.lower() for arg in sys.argv]:
    velocity = -4.0
else:
    velocity = 4.0

# background image name, for the visualization
img_name = '../share/hmm.png'

# pygame initialization
pygame.init()
width, height = 600, 120
screen = pygame.display.set_mode((width, height))

Particle = namedtuple('Particle', ['x', 'w'])
num_particles = 2000
particles = [random.randint(0, 599) for i in xrange(num_particles)]


''' 
Based on p. 252 table 8.2
Algorithm MCL(X_t-1, u_t, z_t, m)
where   X_t-1   is the previous set of particles,
        u_t     is the control (motion, in this case)
        z_t     is the measurement
        m       is unknown... (?)
'''

def MCL(P_prev, control, measurement):

    def left_boundaries(x):
        if x <= 0:
            return random.randint(0, 599)
        else:
            return x 
    def right_boundaries(x):
        if x >= 599:
            return random.randint(0, 599)
        else:
            return x
    
    boundaries = left_boundaries if velocity < 0 else right_boundaries

    # movement model
    P_prime = [boundaries(p + random.gauss(control, 0.75)) for p in P_prev]

    # determine location probability function
    prob_x = smm.p_door if measurement == 'Door' else smm.p_wall
    # measurement model
    W_prime = [prob_x(p/10.0) for p in P_prime]

    # Numpy solution. An equivalent without using numpy is commented out below.
    # cut the deck
    sumstart = random.randint(0, num_particles-1)
    W_prime = np.array(W_prime[sumstart:] + W_prime[:sumstart],
                       dtype = float)
    P_prime = P_prime[sumstart:] + P_prime[:sumstart]

    # list of cumulative sums
    cumsum = W_prime.cumsum() 
    w_sum = cumsum[-1]
    cumsum /= w_sum

    uniform_samples = np.random.random(num_particles)
    ids = cumsum.searchsorted(uniform_samples, side='right')
    P = [P_prime[i] for i in ids]
    
    # Equivalent to the above starting at sumstart, withou numpy
    '''
    # find the start index
    sumstart = random.randint(0, num_particles-1)
    W_prime = W_prime[sumstart:] + W_prime[:sumstart]
    P_prime = P_prime[sumstart:] + P_prime[:sumstart]

    # Calculate cumsum.
    w_sum = 0
    cumsum = []
    for w in W_prime:
        w_sum += w
        cumsum.append(w)

    # find starting value
    w_val = random.random()*cumsum[0]
    # find increment value
    w_inc = w_sum/num_particles

    i = 0
    P = []

    cumsum_len = len(cumsum)
    noise = num_particles - cumsum_len
    while i < cumsum_len:
        P.append(P_prime[i])
        w_val += w_inc
        while i < cumsum_len and w_val > cumsum[i]:
            i += 1
    P += [random.randint(0, 599) for i in xrange(noise)]
    '''

    # store P in the global particles variable
    global particles
    particles = P

    return

def visualize(background = None):
    global screen, width, height

    # draw background
    screen.fill((0, 0, 0))
    if background is not None:
        screen.blit(background, (0, 0))

    # draw bins
    scale = 02
    bins = np.array([0.0 for i in xrange(600)])
    for p in particles:
        b = int(p)
        diff = p - b 
        bins[b] += (1.-diff)*scale 
        if b < 599:
            bins[b+1] += diff*scale

    for i, b in enumerate(bins):
        pygame.draw.line(screen, (255, 0, 0), (i, height-b), (i, height))
    '''
    for p in particles:
        pygame.draw.line(screen, (255, 0, 0), (p, height-45), (p, height-35))
    '''

    pygame.display.flip()
    return
    
def visualize_loop(background = None):
    while True:
        visualize(background)
        sleep(.1)
    return

def sensorreading_handler(sensor_reading):
    MCL(particles, velocity, sensor_reading.data)
    return

def issue_cmd_loop():
    while True:
        controller.publish(cmd)
        sleep(0.1)
    return  

def print_data(reading):
    print reading.data

if __name__ == '__main__':
    random.seed()

    rospy.init_node('particle_filter')

    controller = rospy.Publisher('robot/cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = velocity 
    cmd_thread = threading.Thread(target = issue_cmd_loop)
    cmd_thread.start()

    rospy.Subscriber('robot/wall_door_sensor', String, sensorreading_handler)
    #rospy.Subscriber('robot/wall_door_sensor', String, print_data) 

    bg_img_name = os.path.abspath(img_name)

    try:
        raw_img = pygame.image.load(bg_img_name)
        img = raw_img.convert()
    except pygame.error, message:
        img = None

    vis_thread = threading.Thread(target = visualize_loop, args=(img,))
    vis_thread.start()

    rospy.spin()
