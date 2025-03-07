from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49 # intersection size is doubled bcause roundabouts require more area to implement than a traditional four-way intersection
        island_width = 2
        length = 100 
        radius = 18

        self.vehicle_rate = 20 # vehicle spawn rate 
        self.pedestrian_rate = 40 # Increasing the rate 2 times before
        self.speed_variance = 2.5 
        self.v = 17/1.5 # vehicle speed is two-third of the default
        self.pedestrian_speed = 17/6 # reduce 4 times 
        self.bike_speed = self.pedestrian_speed * 3
        self.pedestrian_colour = (0, 0, 255) # Identify pedestrians by blue colour
        self.bike_colour = (255, 105, 180) # Identify bikes by pink colour
        self.self_driving_vehicle_proportion = 0.5 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5 # Self-driving vehicles can have 1.5 times speed than a non self-driving vehicle due to their better reaction time and overall control
        self.bike_proportion = 0.5 #number between 0 and 1, 0 means no bikes, 1 means entirely bikes
        self.pedestrian_w = 1.75 / 3 # reduce the width 3 times
        self.pedestrian_l = 4 / 3 # reduce the length 3 times
        self.bike_w = 1.75 / 2 # reduce the width 2 times
        self.bike_l = 4 / 2 # reduce the length 2 times
        if self.bike_proportion == 1:
            self.bike_speed = self.bike_speed * 1.5

        #entrance 0-7
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) #0
        self.sim.create_segment((lane_space*3/2 + island_width/2, length+intersection_size/2), (lane_space*3/2+island_width/2, intersection_size/2)) #1
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) #2
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, - lane_space*3/2 - island_width/2)) #3
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) #4
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2)) #5 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) #6
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2)) #7
        # # #exit 8-15
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2)) #8
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2)) #9
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2)) #10
        self.sim.create_segment((intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2)) #11
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2)) #12
        self.sim.create_segment((lane_space*3/2 + island_width/2, -intersection_size/2), (lane_space*3/2 + island_width/2, -length-intersection_size/2)) #13
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2)) #14
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length - intersection_size/2, -lane_space*3/2 - island_width/2)) #15

        #corners 16-23
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2)) #16
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius+lane_space),(radius+lane_space,radius+lane_space),(radius+lane_space,lane_space + island_width/2)) # South-East #17
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius)) #18
        self.sim.create_quadratic_bezier_curve((radius+lane_space,-lane_space - island_width/2),(radius+lane_space,-radius-lane_space),(lane_space + island_width/2,-radius-lane_space)) # North-East #19
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2)) #20
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius-lane_space),(-radius-lane_space,-radius-lane_space),(-radius-lane_space,-lane_space - island_width/2)) # North-West #21
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius)) #22
        self.sim.create_quadratic_bezier_curve((-radius-lane_space,lane_space + island_width/2),(-radius-lane_space,radius+lane_space),(-lane_space - island_width/2, radius+lane_space)) # South-West #23
        
        # #connectors 24-31
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2)) # EAST closer to centre #24
        self.sim.create_segment((radius+lane_space,lane_space + island_width/2),(radius+lane_space,-lane_space - island_width/2)) #25
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius)) # NORTH closer to centre #26
        self.sim.create_segment((lane_space + island_width/2,-radius-lane_space),(-lane_space - island_width/2,-radius-lane_space)) #27
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2)) # WEST closer to centre #28
        self.sim.create_segment((-radius-lane_space,-lane_space - island_width/2),(-radius-lane_space,lane_space + island_width/2)) #29
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius)) # SOUTH closer to centre #30
        self.sim.create_segment((-lane_space - island_width/2, radius+lane_space),(lane_space + island_width/2, radius+lane_space)) #31

        #turn into corners 32-39
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius)) # SOUTH right hand side (when approaching the roundabout) closer to centre #32
        self.sim.create_quadratic_bezier_curve((lane_space*3/2+island_width/2, intersection_size/2),(lane_space*3/2+island_width/2, radius+lane_space*3/2),(lane_space*3/2 + island_width/2, radius+lane_space*3/2)) #33
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2)) # EAST right hand side (when approaching the roundabout) closer to centre #34
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2),(intersection_size/2, -lane_space*3/2 - island_width/2),(radius+3/2*lane_space,-lane_space*3/2 - island_width/2)) #35
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius)) # NORTH right hand side (when approaching the roundabout) closer to centre #36
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, -intersection_size/2),(-lane_space*3/2 - island_width/2, -radius),(-lane_space*3/2 - island_width/2,-radius)) #37
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2)) # WEST right hand side (when approaching the roundabout) closer to centre #38
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2),(-intersection_size/2,lane_space*3/2 + island_width/2),(-radius,lane_space*3/2 + island_width/2)) #39

        #turn to exit 40-47
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2)) # EAST right hand side (when moving away from the roundabout) closer to centre #40
        self.sim.create_quadratic_bezier_curve((radius,lane_space*3/2 + island_width/2),(radius,lane_space*3/2 + island_width/2),(intersection_size/2, lane_space*3/2 + island_width/2)) #41
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2)) # NORTH right hand side (when moving away from the roundabout) closer to centre #42
        self.sim.create_quadratic_bezier_curve((lane_space*2 + island_width/2,-radius),(lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2, -intersection_size/2)) #43 
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2)) # WEST right hand side (when moving away from the roundabout) closer to centre #44
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space*2 - island_width/2),(-radius,-lane_space*3/2 - island_width/2),(-intersection_size/2, -lane_space*3/2 - island_width/2)) #45
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2)) # SOUTH right hand side (when moving away from the roundabout) closer to centre #46
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, radius),(-lane_space*3/2 - island_width/2,radius),(-lane_space*3/2 - island_width/2, intersection_size/2)) #47

        # entry to tunnel for any emergency vehicles going to the hospital
        self.sim.create_segment((lane_space*3/2 + island_width/2, length + intersection_size/2), (lane_space*3/2 + island_width/2, length/2 + intersection_size/2)) #48 # From SOUTH to NORTH
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2, length/2 + intersection_size/2),(lane_space*3/2 + island_width/2,length/2),(lane_space*3 + island_width/2, length/2)) #49

        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2 + length/2, - lane_space*3/2 - island_width/2)) #50 # From EAST to WEST
        self.sim.create_quadratic_bezier_curve((intersection_size/2 + length/2, -lane_space - island_width/2),(intersection_size/2 + length/2,-lane_space*5/2 - island_width/2),(length/2 + intersection_size/2 - lane_space*3, -lane_space*5/2 - island_width/2)) #51

        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2 - length/2)) #52 # From NORTH to SOUTH
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, -intersection_size/2 - length/2), (-lane_space*3/2 - island_width/2,-length/2),(-lane_space*3 - island_width/2, -length/2)) #53

        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2 -length/2, lane_space*3/2 + island_width/2)) #54 # From WEST to EAST
        self.sim.create_quadratic_bezier_curve((-intersection_size/2 - length/2, lane_space + island_width/2),(-intersection_size/2 - length/2, lane_space*5/2 + island_width/2),(-length/2 - intersection_size/2 + lane_space*3, lane_space*5/2 + island_width/2)) #55


        # exit from tunnel for any emergency vehicles moving away from the hospital
        self.sim.create_quadratic_bezier_curve((-lane_space*5/2 - island_width/2, length/2 + intersection_size/2 - lane_space*3),(-lane_space*5/2 - island_width/2,intersection_size/2 + length/2),(-lane_space*3/2 - island_width/2, intersection_size/2 + length/2)) #56
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2 + length/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2)) #57

        self.sim.create_quadratic_bezier_curve((intersection_size/2+length/2-lane_space*3, lane_space*5/2 + island_width/2),(intersection_size/2+length/2-lane_space*3,lane_space*3/2 + island_width/2),(intersection_size/2+length/2, lane_space + island_width/2)) #58
        self.sim.create_segment((length/2+intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2)) #59

        self.sim.create_quadratic_bezier_curve((lane_space*5/2 + island_width/2, -length/2 - intersection_size/2 + lane_space*3),(lane_space*5/2 + island_width/2,-intersection_size/2 - length/2),(lane_space*3/2 + island_width/2, -length/2 - intersection_size/2)) #60
        self.sim.create_segment((lane_space*3/2 + island_width/2, -length/2 - intersection_size/2), (lane_space*3/2 + island_width/2, -intersection_size/2 - length)) #61

        self.sim.create_quadratic_bezier_curve((-intersection_size/2 - length/2+lane_space*3, -lane_space*5/2 - island_width/2),(-intersection_size/2-length/2+lane_space*3, -lane_space*3/2 - island_width/2),(-length/2 - intersection_size/2, -lane_space - island_width/2)) #62
        self.sim.create_segment((-length/2 - intersection_size/2, -lane_space*3/2 - island_width/2), (-intersection_size/2 -length, -lane_space*3/2 - island_width/2)) #63

        # For bike lanes lead to underpass
        self.sim.create_segment((lane_space*5/2 + island_width/2, length+intersection_size/2), (lane_space*5/2+island_width/2, length/2+intersection_size/2)) #64  
        self.sim.create_segment((length + intersection_size/2, -lane_space*5/2 - island_width/2), (length/2+3/2*intersection_size/2, - lane_space*5/2 - island_width/2)) #65
        self.sim.create_segment((-lane_space*5/2 - island_width/2, -length - intersection_size/2), (-lane_space*5/2 - island_width/2, -length/2-intersection_size/2)) #66 
        self.sim.create_segment((-length - intersection_size/2, lane_space*5/2 + island_width/2), (-length/2-3/2*intersection_size/2, lane_space*5/2 + island_width/2)) #67

        self.sim.create_segment((-lane_space*5/2 - island_width/2, length+intersection_size/2), (-lane_space*5/2 - island_width/2, length*3/5 + intersection_size/2)) #68
        self.sim.create_segment((length + intersection_size/2, lane_space*5/2 + island_width/2), (length/2+intersection_size/2, lane_space*5/2 + island_width/2)) #69
        self.sim.create_segment((lane_space*5/2 + island_width/2, -length-intersection_size/2), (lane_space*5/2 + island_width/2, -length*3/5-intersection_size/2)) #70
        self.sim.create_segment((-length-intersection_size/2, -lane_space*5/2 - island_width/2), (-length/2 - intersection_size/2, -lane_space*5/2 - island_width/2)) #71

        # For pedestrian walks lead to underpass
        self.sim.create_segment((lane_space*7/2 + island_width/2, length+intersection_size/2), (lane_space*7/2+island_width/2, length/2+intersection_size/2)) #72
        self.sim.create_segment((length + intersection_size/2, -lane_space*7/2 - island_width/2), (length/2+3/2*intersection_size/2, - lane_space*7/2 - island_width/2)) #73
        self.sim.create_segment((-lane_space*7/2 - island_width/2, -length - intersection_size/2), (-lane_space*7/2 - island_width/2, -length/2-intersection_size/2)) #74
        self.sim.create_segment((-length - intersection_size/2, lane_space*7/2 + island_width/2), (-length/2-3/2*intersection_size/2, lane_space*7/2 + island_width/2)) #75

        self.sim.create_segment((-lane_space*7/2 - island_width/2, length+intersection_size/2), (-lane_space*7/2 - island_width/2, length*3/5 + intersection_size/2)) #76
        self.sim.create_segment((length + intersection_size/2, lane_space*7/2 + island_width/2), (length/2+intersection_size/2, lane_space*7/2 + island_width/2)) #77
        self.sim.create_segment((lane_space*7/2 + island_width/2, -length-intersection_size/2), (lane_space*7/2 + island_width/2, -length*3/5-intersection_size/2)) #78
        self.sim.create_segment((-length-intersection_size/2, -lane_space*7/2 - island_width/2), (-length/2 - intersection_size/2, -lane_space*7/2 - island_width/2)) #79
        self.vg = VehicleGenerator({


            'vehicles': [
                # # 'path': [entry, turn_into_corners, corners, turn_to_exit, exit]
                (1, {'path': [0, 32, 16, 40, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 42, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 44, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 28, 22, 46, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),

                (1, {'path': [1, 33, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 29, 23, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(0, 225, 0)}),

                (1,{'path': [2, 34, 18, 42, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 44, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 46, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 30, 16, 40, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),

                (1, {'path': [3, 35, 19, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 31, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),

                (1, {'path': [4, 36, 20, 44, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 46, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 40, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 24, 18, 42, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),

                (1, {'path': [5, 21, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [5, 21, 29, 23, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [5, 21, 29, 23, 31, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [5, 21, 29, 23, 31, 17, 25, 19, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),                
                
                (1, {'path': [6, 38, 22, 46, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 40, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 42, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 26, 20, 44, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),

                (1, {'path': [7, 23, 8],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [7, 23, 31, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [7, 23, 31, 17, 25, 19, 12],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
                (1, {'path': [7, 23, 31, 17, 25, 19, 27, 21, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (0, 225, 0)}),
            ], 'vehicle_rate': self.vehicle_rate*(1-self.self_driving_vehicle_proportion)
        }
        )

        # Self-driving vehicles
        self.sdvg = VehicleGenerator({

            'vehicles': [
                # 'path': [entry, turn_into_corners, corners, turn_to_exit, exit]
                (1, {'path': [0, 32, 16, 40, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 42, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 44, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 28, 22, 46, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),

                (1, {'path': [1, 33, 17, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 29, 23, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4,'colour':(225, 0, 0)}),

                (1, {'path': [2, 34, 18, 42, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 44, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 46, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 30, 16, 40, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),

                (1, {'path': [3, 35, 19, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 31, 17, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),

                (1, {'path': [4, 36, 20, 44, 14],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 46, 8],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 40, 10],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 24, 18, 42, 12],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),

                (1, {'path': [5, 21, 14],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [5, 21, 29, 23, 8],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [5, 21, 29, 23, 31, 17, 10],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),
                (1, {'path': [5, 21, 29, 23, 31, 17, 25, 19, 12],'v_max': self.v, 'T' : 0.1,'s0' : 8, 'colour': (225, 0, 0)}),                
                
                (1, {'path': [6, 38, 22, 46, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 40, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 42, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 26, 20, 44, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),

                (1, {'path': [7, 23, 8],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [7, 23, 31, 17, 10],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [7, 23, 31, 17, 25, 19, 12],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
                (1, {'path': [7, 23, 31, 17, 25, 19, 27, 21, 14],'v_max': self.v, 'T' : 0.1,'s0' : 4, 'colour': (225, 0, 0)}),
            ], 'vehicle_rate': self.vehicle_rate*self.self_driving_vehicle_proportion 
        }
        )

        # Add emergency medical services vehicle
        self.emsvg = VehicleGenerator({

            # Emergency medical services' vehicles move 1.5 times faster than default speed (default speed = 17 m/s)
            'vehicles': [
                (1, {'path': [48, 49],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(255, 255, 0)}),
                (1, {'path': [50, 51],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(255, 255, 0)}),
                (1, {'path': [52, 53],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(255, 255, 0)}),
                (1, {'path': [54, 55],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(255, 255, 0)}),

                (1, {'path': [56, 57],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 225, 0)}),
                (1, {'path': [58, 59],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 225, 0)}),
                (1, {'path': [60, 61],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 225, 0)}),
                (1, {'path': [62, 63],'v_max': self.v*1.5+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 225, 0)}),

            ], 'vehicle_rate': self.vehicle_rate/5 # Assuming 1 out of 5 vehicles are patient-carrying vehicles and are going to use the tunnel
        }
        )

        #regular pedestrian generator
        self.pg = PedestrianGenerator({
            #The first variable: 1 defines the weight if the pedestrian; the higher the weight the more likely that type of pedestrian will generate
            # 'path' defines the order of segments the pedestrian will drive over
            #'v_max' defines the fastest speed a pedestrian can drive at
            #All regular non-self driving pedestrians will have green color

            'pedestrians': [
                (1, {'path': [72], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [73], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [74], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [75], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),

                (1, {'path': [76], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [77], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [78], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),
                (1, {'path': [79], 'v_max': self.pedestrian_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': self.pedestrian_colour, 'l': self.pedestrian_l, 'w': self.pedestrian_w}),

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
                (1, {'path': [71], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [70], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [69], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [68], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),

                (1, {'path': [67], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [66], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [65], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),
                (1, {'path': [64], 'v_max': self.bike_speed + 2*self.speed_variance*np.random.random() -self.speed_variance, 'T' : 0.75,'s0' : 4, 'colour':self.bike_colour, 'l': self.bike_l, 'w': self.bike_w}),

                ], 'pedestrian_rate' : self.pedestrian_rate*self.bike_proportion 
            })


        #adding three types of vehicle generators
        self.sim.add_vehicle_generator(self.vg) # regular vehicles = green
        self.sim.add_vehicle_generator(self.sdvg) # self-driving vehicles = red
        self.sim.add_vehicle_generator(self.emsvg) # patient carrying vehicles = yellow
        self.sim.add_pedestrian_generator(self.pg) # pedestrians = blue
        self.sim.add_pedestrian_generator(self.bg) # bikes = pink

        # self.sim.define_interfearing_paths([entry_0,turn_into_corners_0],[connectors_7,corners_0],turn=True)
        # self.sim.define_interfearing_paths([entry_2,turn_into_corners_2],[connectors_0,corners_2],turn=True)
        self.sim.define_interfearing_paths([0,32],[30,16],turn=True)
        self.sim.define_interfearing_paths([2,34],[24,18],turn=True)
        self.sim.define_interfearing_paths([4,36],[26,20],turn=True)
        self.sim.define_interfearing_paths([6,38],[28,22],turn=True)
    
        # Uncomment to add the traffic signal
        # self.sim.create_traffic_signal([self.sim.segments[0],self.sim.segments[1], self.sim.segments[4], self.sim.segments[5]],[self.sim.segments[2],self.sim.segments[3],self.sim.segments[6], self.sim.segments[7]])

    def get_sim(self):
        return self.sim