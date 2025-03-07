from .vehicle_generator import VehicleGenerator
from .pedestrian_generator import PedestrianGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.segment import Segment
from .vehicle import Vehicle
from .pedestrian import Pedestrian
from .geometry.traffic_signal import TrafficSignal
import numpy as np


class Simulation:
    def __init__(self):
        self.segments = []
        self.vehicles = {}
        self.pedestrians = {}
        self.vehicle_generator = []
        self.pedestrian_generator = []
        self.traffic_signals = []

        self.t = 0.0
        self.frame_count = 0
        self.dt = 1/60
        self.interfearing_paths = []
        self.vehicle_times = []
        self.pedestrian_times = []



    def define_interfearing_paths(self, interfearing, other,turn):
        #interfearing : [entrance segment reference number to interfearing segment, interfearing segment reference number  (usually a turn)]
        #other: [entrance segment reference number to segment that is interfeared with, segment reference number that is interfeared with]
        #turn a boolean value that is true if the interfearing segment is a turn
        self.interfearing_paths.append([interfearing, other,turn])
    

    def add_vehicle(self, veh):
        self.vehicles[veh.id] = veh
        if len(veh.path) > 0:
            self.segments[veh.path[0]].add_vehicle(veh)

    def add_pedestrian(self, ped):
        self.pedestrians[ped.id] = ped
        if len(ped.path) > 0:
            self.segments[ped.path[0]].add_pedestrian(ped)
    
    def add_segment(self, seg):
        self.segments.append(seg)

    def add_vehicle_generator(self, gen):
        self.vehicle_generator.append(gen)
  
    def add_pedestrian_generator(self, gen):
        self.pedestrian_generator.append(gen)


    def create_pedestrian(self, **kwargs):
        ped = Pedestrian(kwargs)
        self.add_pedestrian(ped)

    def create_segment(self, *args):
        seg = Segment(args)
        self.add_segment(seg)

    def create_quadratic_bezier_curve(self, start, control, end):
        cur = QuadraticCurve(start, control, end)
        self.add_segment(cur)

    def create_cubic_bezier_curve(self, start, control_1, control_2, end):
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)

    def create_vehicle_generator(self, **kwargs):
        gen = VehicleGenerator(kwargs)
        self.add_vehicle_generator(gen)
        
    def create_pedestrian_generator(self, **kwargs):
        gen = PedestrianGenerator(kwargs)
        self.add_pedestrian_generator(gen)

    def create_traffic_signal(self, *args):
        sig = TrafficSignal(args)
        self.add_traffic_signal(sig)
    
    def add_traffic_signal(self, sig):
        self.traffic_signals.append(sig)

    def run(self, steps):
        for _ in range(steps):
            self.update()


    
    def get_average_vehicle_time(self):
        return np.average(self.vehicle_times)
    
    def get_average_pedestrian_time(self):
        return np.average(self.pedestrian_times)
    
    def get_pedestrian_time_variance(self):
       
        return np.var(self.pedestrian_times)
    
    def update(self):
        
        #Update signals
        for signal in self.traffic_signals:
            signal.update(self)
        # Update vehicles
        total_cars  = 0
        total_pedestrians = 0
        for segment in self.segments:
            total_cars += len(segment.vehicles)
        
            if len(segment.vehicles) != 0:
                self.vehicles[segment.vehicles[0]].update(None, self.dt)

            
                if segment.traffic_signal_state:
                    # If traffic signal is green or doesn't exist
                    # Then let vehicles pass    
                    self.vehicles[segment.vehicles[0]].unstop()
                    #Check if any cars are trying to drive on an intefearing path that is in use
                    for seg in self.interfearing_paths:
                        if ((seg[0][0] == self.vehicles[segment.vehicles[0]].path[self.vehicles[segment.vehicles[0]].current_road_index] and seg[0][1] == self.vehicles[segment.vehicles[0]].path[self.vehicles[segment.vehicles[0]].current_road_index + 1]) ) and (len(self.segments[seg[1][1]].vehicles) != 0)\
                        and self.vehicles[segment.vehicles[0]].x >= segment.get_length() - 8:
                            self.vehicles[segment.vehicles[0]].stop()
                            break
                        elif (seg[1][0] == self.vehicles[segment.vehicles[0]].path[self.vehicles[segment.vehicles[0]].current_road_index] and seg[1][1] == self.vehicles[segment.vehicles[0]].path[self.vehicles[segment.vehicles[0]].current_road_index + 1]) and (len(self.segments[seg[0][1]].vehicles) != 0)\
                        and self.vehicles[segment.vehicles[0]].x >= segment.get_length() - 8 :
                            self.vehicles[segment.vehicles[0]].stop()
                            break
                                
                        elif seg[2] and seg[0][1] == self.vehicles[segment.vehicles[0]].path[self.vehicles[segment.vehicles[0]].current_road_index] and (len(self.segments[seg[1][1]].vehicles) != 0)\
                        and segment.get_length()/5<self.vehicles[segment.vehicles[0]].x < 3*segment.get_length()/7 :
                            self.vehicles[segment.vehicles[0]].stop()
                            break
                    

                        
                else:
                    # If traffic signal is red
                    
                    if self.vehicles[segment.vehicles[0]].x >= segment.get_length() - segment.traffic_signal.slow_distance:
                        # Slow vehicles in slowing zone
                        self.vehicles[segment.vehicles[0]].slow(segment.traffic_signal.slow_factor)
                    if self.vehicles[segment.vehicles[0]].x >= segment.get_length() - segment.traffic_signal.stop_distance and\
                    self.vehicles[segment.vehicles[0]].x <= segment.get_length() - segment.traffic_signal.stop_distance / 2:
                        # Stop vehicles in the stop zone
                        self.vehicles[segment.vehicles[0]].stop()
            for i in range(1, len(segment.vehicles)):
                self.vehicles[segment.vehicles[i]].update(self.vehicles[segment.vehicles[i-1]], self.dt)

            
        for segment in self.segments:
            total_pedestrians += len(segment.pedestrians)
        
            if len(segment.pedestrians) != 0:
                self.pedestrians[segment.pedestrians[0]].update(None, self.dt)

            
                if segment.traffic_signal_state:
                    # If traffic signal is green or doesn't exist
                    # Then let pedestrians pass    
                    self.pedestrians[segment.pedestrians[0]].unstop()
                    #Check if any cars are trying to drive on an intefearing path that is in use
                    for seg in self.interfearing_paths:
                        if ((seg[0][0] == self.pedestrians[segment.pedestrians[0]].path[self.pedestrians[segment.pedestrians[0]].current_road_index] and seg[0][1] == self.pedestrians[segment.pedestrians[0]].path[self.pedestrians[segment.pedestrians[0]].current_road_index + 1]) ) and (len(self.segments[seg[1][1]].pedestrians) != 0)\
                        and self.pedestrians[segment.pedestrians[0]].x >= segment.get_length() - 8:
                            self.pedestrians[segment.pedestrians[0]].stop()
                            break
                        elif (seg[1][0] == self.pedestrians[segment.pedestrians[0]].path[self.pedestrians[segment.pedestrians[0]].current_road_index] and seg[1][1] == self.pedestrians[segment.pedestrians[0]].path[self.pedestrians[segment.pedestrians[0]].current_road_index + 1]) and (len(self.segments[seg[0][1]].pedestrians) != 0)\
                        and self.pedestrians[segment.pedestrians[0]].x >= segment.get_length() - 8 :
                            self.pedestrians[segment.pedestrians[0]].stop()
                            break
                                
                        elif seg[2] and seg[0][1] == self.pedestrians[segment.pedestrians[0]].path[self.pedestrians[segment.pedestrians[0]].current_road_index] and (len(self.segments[seg[1][1]].pedestrians) != 0)\
                        and segment.get_length()/5<self.pedestrians[segment.pedestrians[0]].x < 3*segment.get_length()/7 :
                            self.pedestrians[segment.pedestrians[0]].stop()
                            break
                    

                        
                else:
                    # If traffic signal is red
                    
                    if self.pedestrians[segment.pedestrians[0]].x >= segment.get_length() - segment.traffic_signal.slow_distance:
                        # Slow pedestrians in slowing zone
                        self.pedestrians[segment.pedestrians[0]].slow(segment.traffic_signal.slow_factor)
                    if self.pedestrians[segment.pedestrians[0]].x >= segment.get_length() - segment.traffic_signal.stop_distance and\
                    self.pedestrians[segment.pedestrians[0]].x <= segment.get_length() - segment.traffic_signal.stop_distance / 2:
                        # Stop pedestrians in the stop zone
                        self.pedestrians[segment.pedestrians[0]].stop()
            for i in range(1, len(segment.pedestrians)):
                self.pedestrians[segment.pedestrians[i]].update(self.pedestrians[segment.pedestrians[i-1]], self.dt)


        # Check roads for out of bounds vehicle

        for segment in self.segments:
            # If road has no vehicles, continue
            if len(segment.vehicles) == 0: continue
            # If not
            vehicle_id = segment.vehicles[0]
            vehicle = self.vehicles[vehicle_id]
            # If first vehicle is out of road bounds
            if vehicle.x >= segment.get_length():
                # If vehicle has a next road
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    # Update current road to next road
                    vehicle.current_road_index += 1
                    # Add it to the next road
                    next_road_index = vehicle.path[vehicle.current_road_index]
                    self.segments[next_road_index].vehicles.append(vehicle_id)
                elif not vehicle.pedestrian:
                    self.vehicle_times.append(vehicle.time)
 
                # Reset vehicle properties
                vehicle.x = 0
                # In all cases, remove it from its road
                segment.vehicles.popleft() 

        # Check roads for out of bounds pedestrian

        for segment in self.segments:
            # If road has no pedestrians, continue
            if len(segment.pedestrians) == 0: continue
            # If not
            vehicle_id = segment.pedestrians[0]
            vehicle = self.pedestrians[vehicle_id]
            # If first vehicle is out of road bounds
            if vehicle.x >= segment.get_length():
                # If vehicle has a next road
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    # Update current road to next road
                    vehicle.current_road_index += 1
                    # Add it to the next road
                    next_road_index = vehicle.path[vehicle.current_road_index]
                    self.segments[next_road_index].pedestrians.append(vehicle_id)
                elif not vehicle.pedestrian:
                    self.vehicle_times.append(vehicle.time)
 
                # Reset vehicle properties
                vehicle.x = 0
                # In all cases, remove it from its road
                segment.pedestrians.popleft() 

        # Update vehicle generators
        for i in range(len(self.vehicle_generator)):
            try:
                self.vehicle_generator[i].update(self)
            except:
        
                self.vehicle_generator.pop(i)
                break

        # Update pedestrian generators
        for i in range(len(self.pedestrian_generator)):
            try:
                self.pedestrian_generator[i].update(self)
            except:
        
                self.pedestrian_generator.pop(i)
                break
        
        


        # Increment time
        self.t += self.dt
        self.frame_count += 1

