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

        self.vehicle_rate = 20/5 # Vehicle rate is one-fifth of the default value used in base_intersection.py because the road is only for Emergency vehicle's use
        self.speed_variance = 2.5
        self.v = 17*1.5 # The vehicle speed is 1.5 times the default speed (17 m/s)
        self.self_driving_vehicle_proportion = 0.5 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5 # Self-driving vehicles can have 1.5 times speed than a non self-driving vehicle due to their better reaction time and overall control

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
    
        # Add emergency medical services vehicle
        self.emsvg = VehicleGenerator({

            'vehicles': [
                # # 'path': [entry, turn_into_corners, corners, turn_to_exit, exit]
                (1, {'path': [0, 32, 16, 40, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance,'colour':(255, 255, 0)}),
                (1, {'path': [3, 35, 19, 27, 21, 14],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 255, 0)}),
                (1, {'path': [5, 21, 29, 23, 31, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 255, 0)}),
                (1, {'path': [7, 23, 31, 17, 10],'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, 'colour': (255, 255, 0)}),

            ], 'vehicle_rate': self.vehicle_rate
        }
        )

        # self.sim.define_interfearing_paths([entry_0,turn_into_corners_0],[connectors_7,corners_0],turn=True)
        # self.sim.define_interfearing_paths([entry_2,turn_into_corners_2],[connectors_0,corners_2],turn=True)
        self.sim.define_interfearing_paths([0,32],[30,16],turn=True)
        self.sim.define_interfearing_paths([2,34],[24,18],turn=True)
        self.sim.define_interfearing_paths([4,36],[26,20],turn=True)
        self.sim.define_interfearing_paths([6,38],[28,22],turn=True)
        self.sim.add_vehicle_generator(self.emsvg)
    

    def get_sim(self):
        return self.sim