
from .pedestrian import Pedestrian

from numpy.random import randint, random

class PedestrianGenerator:
    def __init__(self, config={}):
        # Set default configurations

        self.set_default_config()

        # Update configurations
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        """Set default configuration"""
        self.pedestrian_rate = 10
        self.pedestrians = [
            (1, {})
        ]

        self.last_added_time = 0


    def init_properties(self):
        self.upcoming_pedestrian = self.generate_pedestrian()

    def generate_pedestrian(self):
        """Returns a random pedestrian from self.pedestrians with random proportions"""

        total = sum(pair[0] for pair in self.pedestrians)
        r = randint(1, total+1)
        for (weight, config) in self.pedestrians:
            r -= weight
            if r <= 0:

                ped = Pedestrian(config)

                return ped
    



    def update(self, simulation):
        """Add pedestrians"""

        if simulation.t - self.last_added_time >=  50/ self.pedestrian_rate:

            # If time elasped after last added pedestrian is
            # greater than pedestrian_period; generate a pedestrian
            segment = simulation.segments[self.upcoming_pedestrian.path[0]]      
            if len(segment.pedestrians) == 0\
               or simulation.pedestrians[segment.pedestrians[-1]].x > self.upcoming_pedestrian.s0 + self.upcoming_pedestrian.l:
                # If there is space for the generated pedestrian; add it
                simulation.add_pedestrian(self.upcoming_pedestrian)
                # Reset last_added_time and upcoming_pedestrian
                self.last_added_time = simulation.t
            self.upcoming_pedestrian = self.generate_pedestrian()