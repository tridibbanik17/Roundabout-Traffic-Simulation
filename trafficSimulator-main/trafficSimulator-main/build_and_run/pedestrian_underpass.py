
from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 1 # reduce lane_space by 3.5 times
        intersection_size = 12 # reduce 2 times
        island_width = 4 # increase 2 times
        length = 33 # reduce by ~3 times


#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.pedestrian_rate = 40 # Increasing the rate 2 times before
        self.pedestrian_speed = 17/6 # reduce 6 times 
        self.bike_speed = self.pedestrian_speed * 3
        self.pedestrian_colour = (0, 0, 255) # Identify pedestrians by blue colour
        self.bike_colour = (255, 105, 180) # Identify bikes by pink colour
        self.speed_variance = 1 # reduce speed variance by 2.5 times
        self.bike_proportion = 0.5 #number between 0 and 1, 0 means no bikes, 1 means entirely bikes
        self.pedestrian_w = 1.75 / 3 # reduce the width 3 times
        self.pedestrian_l = 4 / 3 # reduce the length 3 times
        self.bike_w = 1.75 / 2 # reduce the width 2 times
        self.bike_l = 4 / 2 # reduce the length 2 times
        if self.bike_proportion == 1:
            self.bike_speed = self.bike_speed * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#
    #this section defines all the paths that a pedestrian can take
    # SOUTH, EAST, NORTH, WEST
        #INNER, OUTER
        # Intersection in
            #paths 0-7
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        self.sim.create_segment((lane_space*3/2 + island_width/2, length+intersection_size/2), (lane_space*3/2+island_width/2, intersection_size/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, - lane_space*3/2 - island_width/2)) 
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2))
        # Intersection out
            #paths 8-15
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_segment((intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((lane_space*3/2 + island_width/2, -intersection_size/2), (lane_space*3/2 + island_width/2, -length-intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length - intersection_size/2, -lane_space*3/2 - island_width/2))

        # Straight
            #paths 16-23
        self.sim.create_segment((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_segment((lane_space*3/2 + island_width/2, intersection_size/2), (lane_space*3/2 + island_width/2, -intersection_size/2))
        self.sim.create_segment((intersection_size/2, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((intersection_size/2, -lane_space*3/2 - island_width/2), (-intersection_size/2, -lane_space*3/2 - island_width/2))
        self.sim.create_segment((-lane_space/2 - island_width/2, -intersection_size/2), (-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -intersection_size/2), (-lane_space*3/2 - island_width/2, intersection_size/2))
        self.sim.create_segment((-intersection_size/2, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_segment((-intersection_size/2, lane_space*3/2 + island_width/2), (intersection_size/2, lane_space*3/2 + island_width/2))
    
    # SOUTH, EAST, NORTH, WEST
        #Right turn
            #paths 24-27 
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2, intersection_size/2), (lane_space*3/2 + island_width/2, lane_space*3/2 + island_width/2), (intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2), (lane_space*3/2 + island_width/2, -lane_space*3/2 - island_width/2), (lane_space*3/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, -intersection_size/2), (-lane_space*3/2 - island_width/2, -lane_space*3/2 - island_width/2), (-intersection_size/2, -lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2), (-lane_space*3/2 - island_width/2, lane_space*3/2 + island_width/2), (-lane_space*3/2 - island_width/2, intersection_size/2))

        # Left turn
            #paths 28-31
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2), (-lane_space/2 - island_width/2, -lane_space/2 - island_width/2), (-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, -intersection_size/2), (-lane_space/2 - island_width/2, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2), (lane_space/2 + island_width/2, lane_space/2 + island_width/2), (lane_space/2 + island_width/2, -intersection_size/2))
        
    #all interfearing paths
    
        #left turn from the South intersects with the inner and outer straights coming from the North
        self.sim.define_interfearing_paths([0, 28], [4, 20],turn=True)
        self.sim.define_interfearing_paths([0, 28], [5, 21],turn=True)
        #left turn from the Rast intersects with the inner and outer straights coming from the Weat
        self.sim.define_interfearing_paths([2, 29], [6, 22],turn=True)
        self.sim.define_interfearing_paths([2, 29], [7, 23],turn=True)
        #left turn from the North intersects with the inner and outer straights coming from the south
        self.sim.define_interfearing_paths([4, 30], [0, 16],turn=True)
        self.sim.define_interfearing_paths([4, 30], [1, 17],turn=True)
        #left turn from the West intersects with the inner and outer straights coming from the East
        self.sim.define_interfearing_paths([6, 31], [2, 18],turn=True)
        self.sim.define_interfearing_paths([6, 31], [3, 19],turn=True)

        '''
        this section creates pedestrian generators, we have two pedestrian generators one; that creates pedestrians (self.pg)
        and one that creates bikes (self.bg)
        '''
        #regular pedestrian generator
        self.pg = PedestrianGenerator({
            #The first variable: 1 defines the weight if the pedestrian; the higher the weight the more likely that type of pedestrian will generate
            # 'path' defines the order of segments the pedestrian will drive over
            #'v_max' defines the fastest speed a pedestrian can drive at
            #All regular non-self driving pedestrians will have green color

            'pedestrians': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [1, 17, 13], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [1, 24, 11], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [3, 19, 15], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [3, 25, 13], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [5, 21, 9], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [5, 26, 15], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [7, 23, 11], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [7, 27, 9], 'v_max': self.pedestrian_speed+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                ], 'pedestrian_rate' : self.pedestrian_rate*(1-self.bike_proportion) 
            })
        
        
        #bike generator
        self.bg = PedestrianGenerator({
 
            #The first variable: 1 defines the weight if the pedestrian; the higher the weight the more likely that type of pedestrian will generate
            # 'path' defines the order of segments the pedestrian will drive over
            #'v_max' defines the fastest speed a pedestrian can drive at
            #'T' defines the reaction time of the pedestrian, the base is 1
            #'s0' defines the shortest distance a pedestrian is able to drive behind another pedestrian 
            #All self-driving pedestrians will have red color
            'pedestrians': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [0, 16, 12], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [0, 28, 14], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [2, 18, 14], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [2, 29, 8], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [4, 20, 8], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [4, 30, 10], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [6, 22, 10], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [6, 31, 12], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                ], 'pedestrian_rate' : self.pedestrian_rate*self.bike_proportion 
            })
        
        #adding both pedestrian generators
        self.sim.add_pedestrian_generator(self.pg)
        self.sim.add_pedestrian_generator(self.bg)
    
    #this function returns an instance of the simulation defined above
    def get_sim(self):
        return self.sim