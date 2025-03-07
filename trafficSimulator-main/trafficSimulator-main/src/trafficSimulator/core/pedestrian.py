import uuid
import numpy as np
import random as rand


class Pedestrian:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()
        
    def set_default_config(self):    
        self.id = uuid.uuid4()
        self.w = 1.75/2 # reduce the width 2 times
        self.l = 4/3 # reduce the length 3 times
        self.s0 = 2 # reduce the shortest distance behind another one by 4 times
        self.T = 0.5 # Keep the reaction time 2 times faster by reducing it 2 times
        self.v_max = 17/6  # reduce the max speed 6 times
        self.a_max = 5/4 # reduce 4 times
        self.b_max = 4.61/4 # reduce 4 times
        self.time = 0 
        self.path = []
        self.current_road_index = 0
        self.pedestrian = False
        self.colour = (0, 0, 255)

        self.x = 0
        self.v = 17/6
        self.a = 0
        self.stopped = False



    def init_properties(self):
        self.sqrt_ab = 2*np.sqrt(self.a_max*self.b_max)
        self._v_max = self.v_max


    def update(self, lead, dt):
        # Update position and velocity
        self.time += dt
        if self.v + self.a*dt < 0:
            self.x -= 1/2*self.v*self.v/self.a
            self.v = 0
        else:
            self.v += self.a*dt
            self.x += self.v*dt + self.a*dt*dt/2
        
        # Update acceleration
        alpha = 0
        if lead:
            delta_x = lead.x - self.x - lead.l
            delta_v = self.v - lead.v

            alpha = (self.s0 + max(0, self.T*self.v + delta_v*self.v/self.sqrt_ab)) / delta_x

        self.a = self.a_max * (1-(self.v/self.v_max)**4 - alpha**2)

        if self.stopped: 
            self.a = -3*self.b_max*self.v/self.v_max
        

    def stop(self):
        self.stopped = True

    def unstop(self):
        self.stopped = False


    def slow(self, v):
        self.v_max = v


    def unslow(self):
        self.v_max = self._v_max
    
