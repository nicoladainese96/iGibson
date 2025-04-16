# Abstract class to apply high-level actions to the simulator
from abc import ABC, abstractmethod

class SimActionApplier(ABC):
    def __init__(self, env):
        self.env = env

    @abstractmethod
    def go_to(self, target): pass

    @abstractmethod
    def open(self, object_name): pass

    @abstractmethod
    def close(self, object_name): pass

    @abstractmethod
    def grasp(self, object_name): pass

    @abstractmethod
    def place_inside(self, object_a, object_b): pass
        
    @abstractmethod
    def place_on_top(self, object_a, object_b): pass

    
