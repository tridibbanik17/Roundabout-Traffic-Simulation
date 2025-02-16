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

        self.v = 8.5

        #entrance 0-3
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        self.sim.create_segment((lane_space*3/2 + island_width/2, length+intersection_size/2), (lane_space*3/2+island_width/2, intersection_size/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, - lane_space*3/2 - island_width/2)) 
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2))
        #exit4-7
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_segment((intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))
        self.sim.create_segment((lane_space*3/2 + island_width/2, -intersection_size/2), (lane_space*3/2 + island_width/2, -length-intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length - intersection_size/2, -lane_space*3/2 - island_width/2))

        #corners 8-11
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius+lane_space),(radius+lane_space,radius+lane_space),(radius+lane_space,lane_space + island_width/2)) # South-East
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((radius+lane_space,-lane_space - island_width/2),(radius+lane_space,-radius-lane_space),(lane_space + island_width/2,-radius-lane_space)) # North-East
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius-lane_space),(-radius-lane_space,-radius-lane_space),(-radius-lane_space,-lane_space - island_width/2)) # North-West
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((-radius-lane_space,lane_space + island_width/2),(-radius-lane_space,radius+lane_space),(-lane_space - island_width/2, radius+lane_space)) # South-West
        
        #connectors 12-15
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2)) # EAST closer to centre
        self.sim.create_segment((radius+lane_space,lane_space + island_width/2),(radius+lane_space,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius)) # NORTH closer to centre
        self.sim.create_segment((lane_space + island_width/2,-radius-lane_space),(-lane_space - island_width/2,-radius-lane_space)) 
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2)) # WEST closer to centre
        self.sim.create_segment((-radius-lane_space,-lane_space - island_width/2),(-radius-lane_space,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius)) # SOUTH closer to centre
        self.sim.create_segment((-lane_space - island_width/2, radius+lane_space),(lane_space + island_width/2, radius+lane_space))
        #turn into corners 16-19
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius)) # SOUTH right hand side (when approaching the roundabout) closer to centre 
        self.sim.create_quadratic_bezier_curve((lane_space*3/2+island_width/2, intersection_size/2),(lane_space*3/2+island_width/2, radius),(lane_space*3/2 + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2)) # EAST right hand side (when approaching the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2),(radius, -lane_space*3/2 - island_width/2),(radius+lane_space,-lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius)) # NORTH right hand side (when approaching the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, - intersection_size/2),(-lane_space*3/2 - island_width/2, -radius),(-lane_space*3/2 - island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2)) # WEST right hand side (when approaching the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2),(-intersection_size/2,lane_space*3/2 + island_width/2),(-radius,lane_space*3/2 + island_width/2))

        #turn to exit 20-23
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2)) # EAST right hand side (when moving away from the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((radius,lane_space*3/2 + island_width/2),(radius,lane_space*3/2 + island_width/2),(intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2,-radius),(lane_space*3/2 + island_width/2, -intersection_size/2)) # NORTH right hand side (when moving away from the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2)) # WEST right hand side (when moving away from the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space*3/2 - island_width/2),(-radius,-lane_space*3/2 - island_width/2),(-intersection_size/2, -lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2)) # SOUTH right hand side (when moving away from the roundabout) closer to centre
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, radius),(-lane_space*3/2 - island_width/2,radius),(-lane_space*3/2 - island_width/2, intersection_size/2))
    
        self.vg = VehicleGenerator({


            'vehicles': [
                (1, {'path': [0, 16, 8,20,5],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 16, 8,12,9,21,6],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 16, 8,12,9,13,10,22,7],'v_max':self.v,'colour':(225, 0, 0, 80)}),
                (1, {'path': [0, 16, 8,12,9,13,10,14,11,23,4],'v_max':self.v,'colour':(225, 0, 0, 80)}),

                (1,{'path': [1, 17, 9, 21, 6],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [1,17,9,13,10,22,7],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [1, 17, 9,13,10,14,11,23,4],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [1, 17, 9,13,10,14,11,15,8,20,5],'v_max':self.v, 'colour': (225, 0, 0, 80)}),

                (1, {'path': [2, 18, 10, 22, 7],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2,18,10,14,11,23,4],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2,18,10,14,11,15,8,20,5],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [2, 18, 10,14,11,15,8,12,9,21,6],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                
                (1, {'path': [3, 19, 11, 23, 4],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3,19,11,15,8,20,5],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3,19,11,15,8,12,9,21,6],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
                (1, {'path': [3, 19, 11,15,8,12,9,13,10,22,7],'v_max':self.v, 'colour': (225, 0, 0, 80)}),
            ], 'vehicle_rate': 30
        }
        
        )
        self.sim.define_interfearing_paths([0,16],[15,8],turn=True)
        self.sim.define_interfearing_paths([1,17],[12,9],turn=True)
        self.sim.define_interfearing_paths([2,18],[13,10],turn=True)
        self.sim.define_interfearing_paths([3,19],[14,11],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
        #adding the traffic signal
        self.sim.create_traffic_signal([self.sim.segments[0],self.sim.segments[2],self.sim.segments[4], self.sim.segments[6]], [self.sim.segments[1],self.sim.segments[3], self.sim.segments[5], self.sim.segments[7]])

    def get_sim(self):
        return self.sim