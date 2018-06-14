
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

class PP:
    def __init__(self,filename = None):
        self.DT = 0.1 # time tick (s)
        self.WB = 2.9 # wheel base (m)
        self.vehicle = Vehicle()
        self.Lfc = 1 # look-ahead distance
        self.Kf = 0.1 # look forward gain
        self.Kv = 0.1 # speed gain
        self.road = Road()
        if filename is not None:
            self.road = Road(filename)
        self.state = State()

        self.step = 0
        self.reward = 0
        self.eDis = 0
        self.eYaw = 0

        #init pygame
        self.fullscreen = True
        pygame.init()
        self._display = pygame.display.set_mode( (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._line = pygame.display.set_mode( (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.flip()
        self.map_points = self._generate_pointlist()
        self.trajectory = []

        self.oa = []

        def pure_pursuit_control(self,oa):
        
            ind = calc_target_index(state) 
        
            tx = cx[ind]
            ty = cy[ind]
        
            alpha = math.atan2(ty - self.state.y, tx - self.state.x) - self.state.yaw
        
            Lf = self.Kv * self.state.v + self.Lfc 
        
            delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)
        
            return delta

        def calc_target_index(self):
        
            ind, dis = self.road.FindNearest(self.state.x, self.state.y)
            L = 0.0 
            #
            Lf = self.Kv * self.state.v + self.Lfc 
            ncourse = len(self.road.cx)
            # search look ahead target point index
            while Lf > L:
                dx = cx[(ind + 1)%ncourse] - cx[ind%ncourse]
                dy = cx[(ind + 1)%ncourse] - cx[ind%ncourse]
                L += math.sqrt(dx ** 2 + dy ** 2)
                ind += 1
        
            return ind % ncourse

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
        #if keys[K_SPACE] or keys[K_UP] or keys[K_DOWN]:
        #    self.fullscreen = not self.fullscreen
        #if keys[K_RETURN]:
        #    print("pause 5s")
        #    time.sleep(5)
        pygame.display.flip()
        pygame.display.update()

    def _generate_pointlist(self):
        map_points = []
        for (cx,cy) in zip(self.road.cx,self.road.cy):
            map_points.append( (int(cx * PIXEL_DENSITY + FULL_OFFSET_X),int(cy * PIXEL_DENSITY + FULL_OFFSET_Y)) )
        return map_points

    
    def update_state(self,state, a, delta):
        # input check
        if delta >= self.vehicle.MAX_STEER:
            delta = self.vehicle.MAX_STEER
        elif delta <= -self.vehicle.MAX_STEER:
            delta = -self.vehicle.MAX_STEER
    
        state.x = state.x + state.v * math.cos(state.yaw) * self.DT
        state.y = state.y + state.v * math.sin(state.yaw) * self.DT
        state.yaw = state.yaw + state.v / self.vehicle.WB * math.tan(delta) * self.DT
        #state.yaw = pi_2_pi( state.yaw + state.v / self.vehicle.WB * math.tan(delta) * self.DT )
        state.v = state.v + a * self.DT
    
        if state. v > self.vehicle.MAX_SPEED:
            state.v = self.vehicle.MAX_SPEED
        elif state. v < self.vehicle.MIN_SPEED:
            state.v = self.vehicle.MIN_SPEED
    
        return state
    
    
    def log(self):
        pass

#dependent function
def main():
    print(__file__ + " start!!")

    #mpc = MPC()
    #mpc.road = Road(filename = '../log/all.road')
    ppsuit = PP(filename = '../log/all.road')
    #mpc.road.test()
    def do_simulation2(mpc):
        #get state
        #state = State(x=mpc.cx[0], y=mpc.cy[0], yaw=mpc.cyaw[0], v=0.0)
        mpc.state = State(x=mpc.road.cx[0], y=mpc.road.cy[0] + 2, yaw=mpc.road.cyaw[0], v=0.0)
        times = 0.0
        odelta, oa = None, None
        ai,di = 0,0 
        # get new state
        mpc.state = mpc.update_state(mpc.state, ai, di)

        x = [mpc.state.x]
        y = [mpc.state.y]
        yaw = [mpc.state.yaw]
        v = [mpc.state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]


        while times < mpc.MAX_TIME:

            # get current state
            di = mpc.control_steer(mpc.oa)
    
            ai = mpc.oa[0]
    
            # get new state
            mpc.state = mpc.update_state(mpc.state, ai, di)
            times = times + mpc.DT
    
            x.append(mpc.state.x)
            y.append(mpc.state.y)
            yaw.append(mpc.state.yaw)
            v.append(mpc.state.v)
            t.append(times)
            d.append(di)
            a.append(ai)
    
            #if mpc.show_animation:
            if False:
                plt.cla()
                #if ox is not None:
                #    plt.plot(ox, oy, "xr", label="MPC")
                plt.plot(mpc.road.cx, mpc.road.cy, "-r", label="course")
                plt.plot(x, y,  label="trajectory", linewidth=3)
                plt.plot(mpc.road.xref[0, :], mpc.road.xref[1, :], "xk", label="xref")
                mpc.plot_car(mpc.state.x, mpc.state.y ,mpc.state.yaw, steer=di)
                plt.axis("equal")
                plt.grid(True)
                plt.title("speed[km/h]:" + str(round(mpc.state.v*3.6, 2)) +
                          ", yaw:" + str(round(mpc.state.yaw,2)) + 
                          ", steer" + str(round(di,2)))
                plt.pause(0.0001)
            #time.sleep(0.1)
    
        return t, x, y, yaw, v, d, a
    t, x, y, yaw, v, d, a = do_simulation2(mpc)

if __name__ == '__main__':
    main()
