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
sys.path.append("controller/")
from mpc import *
import wrapped_carla_sim as simulator
import random
import numpy as np
import threading
from utils import proPrint
from collections import deque
sys.path.append("libs/")
from timer import *
import time
def Control(game, speed, steer):
    game.new_game()
    timer = Timer()
    while True:
        timer.tick()
        if timer.elapsed_seconds_since_lap() >= 0.1:
            steer.state = game.state()
            ste = steer.compute_steer()
            throttle, brake = speed.compute(target, real)
            game.send_control(steer = ste, throttle = throttle, brake = brake)
            timer.lap()
        steer.render()
        time.sleep(0.025)
    #controller.state = game.state()
    #speedcontrol
    #steercontrol
    #render

    pass

def playGame(args):
    with make_carla_client(args.host, args.port) as client:
        game = simulator.CarlaGame(client, args)
        speed = SpeedController()
        steer = MPC(filename = './log/all.road')
        Control(game, speed, steer)

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
