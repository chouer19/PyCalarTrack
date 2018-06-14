#!/usr/bin/env python
from __future__ import print_function

import argparse
import logging
import random
import time
import math

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

import tensorflow as tf
import cv2
import sys
#sys.path.append("game/")
#import wrapped_flappy_car as game
import wrapped_carla_sim as simulator
import random
import numpy as np
import threading
from utils import proPrint
from collections import deque

GAME = 'angry-car' # the name of the game being played for log files
#ACTIONS = 2 # number of valid actions
ACTIONS = 20 # number of valid actions
GAMMA = 0.99 # decay rate of past observations
OBSERVE = 100000. # timesteps to observe before training
EXPLORE = 3000000. # frames over which to anneal epsilon
EXPLORE = 2000000. # frames over which to anneal epsilon
FINAL_EPSILON = 0.0001 # final value of epsilon
INITIAL_EPSILON = 0.16 # starting value of epsilon
INITIAL_EPSILON = 0.0001 # starting value of epsilon
#INITIAL_EPSILON = 0.2 # starting value of epsilon
REPLAY_MEMORY = 50000 # number of previous transitions to remember
REPLAY_MEMORY = 50000 # number of previous transitions to remember
BATCH = 32 # size of minibatch
FRAME_PER_ACTION = 1
FPS = 20


def Control(game, controller):
    #game.new_game()
    #controller.state = game.state()
    #speedcontrol
    #steercontrol
    #render

def playGame(args):
    with make_carla_client(args.host, args.port) as client:
        game = simulator.CarlaGame(client, args)
        #speedController = SpeedController()
        #controller = Controler()
        #controller.readmap('test.road')
        #Control(game, controller)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-r', '--road',
        metavar='R',
        default='waypoints',
        help='road location of waypoints road')
    argparser.add_argument(
        '-rl', '--road_length',
        metavar='RL',
        default=15,
        type=int,
        help='length of stright roads')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-m', '--map_name',
        metavar='M',
        default='Town01',
        help='plot the map of the current city (needs to match active map in '
             'server, options: Town01 or Town02)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    playGame(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
