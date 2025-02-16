from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49
        island_width = 2
        length = 50 #I have shortened the length of the entrance roads because vehicles base speeds are much lower because they are driving in a round about, however they would be able to drive faster in the entrance.
        radius = 18

        self.v = 12.75

        #entrance 0-7
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) #0
        self.sim.create_segment((lane_space*3/2 + island_width/2, length+intersection_size/2), (lane_space*3/2+island_width/2, intersection_size/2)) #1
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) #2
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, - lane_space*3/2 - island_width/2)) #3
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) #4
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2)) #5 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) #6
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2)) #7
        #exit 8-15
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
        
        #connectors 24-31
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
        self.sim.create_quadratic_bezier_curve((lane_space*3/2+island_width/2, intersection_size/2),(lane_space*3/2+island_width/2, radius),(lane_space*3/2 + island_width/2, radius)) #33
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2)) # EAST right hand side (when approaching the roundabout) closer to centre #34
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2),(radius, -lane_space*3/2 - island_width/2),(radius+lane_space,-lane_space*3/2 - island_width/2)) #35
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius)) # NORTH right hand side (when approaching the roundabout) closer to centre #36
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, - intersection_size/2),(-lane_space*3/2 - island_width/2, -radius),(-lane_space*3/2 - island_width/2,-radius)) #37
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2)) # WEST right hand side (when approaching the roundabout) closer to centre #38
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2),(-intersection_size/2,lane_space*3/2 + island_width/2),(-radius,lane_space*3/2 + island_width/2)) #39

        #turn to exit 40-47
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2)) # EAST right hand side (when moving away from the roundabout) closer to centre #40
        self.sim.create_quadratic_bezier_curve((radius,lane_space*3/2 + island_width/2),(radius,lane_space*3/2 + island_width/2),(intersection_size/2, lane_space*3/2 + island_width/2)) #41
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2)) # NORTH right hand side (when moving away from the roundabout) closer to centre #42
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2, -intersection_size/2)) #43 
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2)) # WEST right hand side (when moving away from the roundabout) closer to centre #44
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space*3/2 - island_width/2),(-radius,-lane_space*3/2 - island_width/2),(-intersection_size/2, -lane_space*3/2 - island_width/2)) #45
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2)) # SOUTH right hand side (when moving away from the roundabout) closer to centre #46
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, radius),(-lane_space*3/2 - island_width/2,radius),(-lane_space*3/2 - island_width/2, intersection_size/2)) #47
    
        self.vg = VehicleGenerator({


            'vehicles': [
                # 'path': [entry, turn_into_corners, corners, turn_to_exit, exit]
                (1, {'path': [0, 32, 16, 40, 10],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 32, 16, 24, 18, 42, 12],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 44, 14],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 32, 16, 24, 18, 26, 20, 28, 22, 46, 8],'v_max':self.v,'colour':(225, 0, 0, 80)}),

                (1, {'path': [1, 33, 17, 41, 11],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [1, 33, 17, 25, 19, 43, 13],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 45, 15],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [1, 33, 17, 25, 19, 27, 21, 29, 23, 47, 9],'v_max':self.v,'colour':(225, 0, 0, 80)}),

                (1,{'path': [2, 34, 18, 42, 12],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2, 34, 18, 26, 20, 44, 14],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 46, 8],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2, 34, 18, 26, 20, 28, 22, 30, 16, 40, 10],'v_max':self.v, 'colour': (225, 0, 0, 80)}),

                (1,{'path': [3, 35, 19, 43, 13],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3, 35, 19, 27, 21, 45, 15],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 47, 9],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3, 35, 19, 27, 21, 29, 23, 31, 17, 41, 11],'v_max':self.v, 'colour': (225, 0, 0, 80)}),

                (1, {'path': [4, 36, 20, 44, 14],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [4, 36, 20, 28, 22, 46, 8],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 40, 10],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [4, 36, 20, 28, 22, 30, 16, 24, 18, 42, 12],'v_max':self.v, 'colour': (225, 0, 0, 80)}),

                (1, {'path': [5, 37, 21, 45, 15],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [5, 37, 21, 29, 23, 47, 9],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [5, 37, 21, 29, 23, 31, 17, 41, 11],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [5, 37, 21, 29, 23, 31, 17, 25, 19, 43, 13],'v_max':self.v, 'colour': (225, 0, 0, 80)}),                
                
                (1, {'path': [6, 38, 22, 46, 8],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [6, 38, 22, 30, 16, 40, 10],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 42, 12],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [6, 38, 22, 30, 16, 24, 18, 26, 20, 44, 14],'v_max':self.v, 'colour': (225, 0, 0, 80)}),

                (1, {'path': [7, 39, 23, 47, 9],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [7, 39, 23, 31, 17, 41, 11],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [7, 39, 23, 31, 17, 25, 19, 43, 13],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [7, 39, 23, 31, 17, 25, 19, 27, 21, 45, 15],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
            ], 'vehicle_rate': 30
        }
        
        )
        # self.sim.define_interfearing_paths([entry_0,turn_into_corners_0],[connectors_7,corners_0],turn=True)
        # self.sim.define_interfearing_paths([entry_2,turn_into_corners_2],[connectors_0,corners_2],turn=True)
        self.sim.define_interfearing_paths([0,32],[30,16],turn=True)
        self.sim.define_interfearing_paths([2,34],[24,18],turn=True)
        self.sim.define_interfearing_paths([4,36],[26,20],turn=True)
        self.sim.define_interfearing_paths([6,38],[28,22],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
        #adding the traffic signal
        self.sim.create_traffic_signal([self.sim.segments[0],self.sim.segments[1], self.sim.segments[4], self.sim.segments[5]],[self.sim.segments[2],self.sim.segments[3],self.sim.segments[6], self.sim.segments[7]])

    def get_sim(self):
        return self.sim