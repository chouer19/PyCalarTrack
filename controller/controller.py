
import sys
sys.path.append("../libs/CubicSpline/")
import numpy as np
import cvxpy
import matplotlib.pyplot as plt
import cubic_spline_planner

sys.path.append("../libs")
from vehicle import *
from road import *

import time
import pygame
from pygame.locals import *

#dependent function
def get_nparray_from_matrix(x):
    return np.array(x).flatten()

BLACK = (0,0,0)
WHITE = (255,255,255)
RED = (255,0,0)
DARKPINK = (255,20,147)
DARKRED = (138,0,0)
PURPLE = (160,32,240)
YELLOW = (255,255,0)
GREEN = (00,255,0)
BLUE = (0,0,255)
LIGHTBLUE = (176,226,255)
ORANGE = (139,69,0)

WIDTH = 1200
HEIGHT = 1000
PIXEL_DENSITY = 2.8

FULL_OFFSET_X = 25
FULL_OFFSET_Y = 25

CAR_WIDTH = 2.4
CAR_LENGTH = 5.5

class MPC:
    def __init__(self,filename = None):
        self.NX = 4  # x = x, y, v, yaw
        self.NU = 2  # a = [accel, steer]
        self.T = 5  # horizon length
        self.HorizonLength = 5
        #self.HorizonLength = 10
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Q = np.diag([1.0, 1.0, 1.0, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix

        # iterative paramter
        self.MAX_ITER = 8  # Max iteration
        self.DU_TH = 0.1  # iteration finish param
        self.N_IND_SEARCH = 10  # Search index number
    
        self.show_animation = True

        self.DT = 0.2 # time tick (s)
        self.DT = 0.1 # time tick (s)

        self.vehicle = Vehicle()
        self.road = Road()
        if filename is not None:
            self.road = Road(filename)
        self.state = State()

        self.oa = [0.0] * self.HorizonLength
        self.odelta = [0.0] * self.HorizonLength

        #init pygame
        self.fullscreen = True
        pygame.init()
        self._display = pygame.display.set_mode( (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._line = pygame.display.set_mode( (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.flip()
        self.map_points = self._generate_pointlist()
        self.trajectory = []

    def pp_steer(self):
        pass

    def stanely_steer(self):
        pass

    def pid_steer(self):
        pass

    def render(self):
        self._line.fill(BLACK)
        self._display.fill(BLACK)
        self.trajectory.append(( int(self.state.x * PIXEL_DENSITY + FULL_OFFSET_X), int(self.state.y * PIXEL_DENSITY + FULL_OFFSET_Y)  ) )
        if len(self.trajectory) > 1000:
            self.trajectory = self.trajectory[5:-1]
        if self.fullscreen:
            pygame.draw.lines(self._line,GREEN,False,self.map_points, 2)
            pygame.draw.circle(self._line, (255,255,255), [int(self.state.x * PIXEL_DENSITY + FULL_OFFSET_X), int(self.state.y * PIXEL_DENSITY + FULL_OFFSET_Y)], 3)
            pygame.draw.lines(self._line, RED, False, \
                                        [ ( int(self.state.x * PIXEL_DENSITY + FULL_OFFSET_X), int(self.state.y * PIXEL_DENSITY + FULL_OFFSET_Y) ),\
                                         ( int(self.state.x * PIXEL_DENSITY + math.cos(self.state.yaw)* 20 + FULL_OFFSET_X), int(self.state.y * PIXEL_DENSITY + math.sin(self.state.yaw)*20 + FULL_OFFSET_Y) )] ,3)
            if len(self.trajectory) > 10:
                pygame.draw.lines(self._line, ORANGE,False,self.trajectory, 2)
            pass
        else:
            
            pass
        pass
        for event in pygame.event.get():
            pass
        keys = pygame.key.get_pressed()
        pygame.display.flip()
        pygame.display.update()

    def _generate_pointlist(self):
        map_points = []
        for (cx,cy) in zip(self.road.cx,self.road.cy):
            map_points.append( (int(cx * PIXEL_DENSITY + FULL_OFFSET_X),int(cy * PIXEL_DENSITY + FULL_OFFSET_Y)) )
        return map_points

    def update_state(self,state, a, delta):
        if delta >= self.vehicle.MAX_STEER:
            delta = self.vehicle.MAX_STEER
        elif delta <= -self.vehicle.MAX_STEER:
            delta = -self.vehicle.MAX_STEER
    
        state.x = state.x + state.v * math.cos(state.yaw) * self.DT
        state.y = state.y + state.v * math.sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.vehicle.WB * math.tan(delta) * self.DT
        state.v = state.v + a * self.DT
    
        if state. v > self.vehicle.MAX_SPEED:
            state.v = self.vehicle.MAX_SPEED
        elif state. v < self.vehicle.MIN_SPEED:
            state.v = self.vehicle.MIN_SPEED
    
        return state
