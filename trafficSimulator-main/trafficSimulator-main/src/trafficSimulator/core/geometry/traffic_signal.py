class TrafficSignal:
    def __init__(self, segments, config={}):
        # Initialize segments
        self.segments = segments
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)
        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        # Divide the state times by 1.5 because roundabout aims to clear traffic congestion sooner than a regular intersection. No left turn delays.
        self.cycle = [(False, True,40), (False,False,44), (True, False,84), (False, False, 88)]
        self.slow_distance = 25
        self.slow_factor = 5
        self.stop_distance = 10


        self.current_cycle_index = 0
        self.last_t = 0
    

    def init_properties(self):
        for i in range(len(self.segments)):
            for segment in self.segments[i]:
                segment.set_traffic_signal(self, i)

    @property
    def current_cycle(self):
        return self.cycle[self.current_cycle_index]

    def update(self, sim):
      
        
        k = int(sim.t) % self.cycle[-1][2]
        for i in range(len(self.cycle)):
            if k < self.cycle[i][2]:
                self.current_cycle_index=i
                break


        