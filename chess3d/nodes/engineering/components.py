from abc import ABC, abstractmethod
from typing import Union
import uuid

from actions import *


class AbstractComponent(ABC):
    """
    # Abstract Component 

    Represents a generic component that is part of a subsystem onboard an agent's Engineering Module

    ### Attributes:
        - name (`str`) : name of the component
        - status (`str`) : current status of the component
        - t (`float` or `int`) : last updated time
        - id (`str`) : identifying number for this task in uuid format
        - min_power (`float`) : amount of power needed to turn on component
    """
    ENABLED = 'ENABLED'
    DISABLED = 'DISABLED'
    CRITICAL = 'CRITICAL'
    FAILED = 'FAILED'

    def __init__(   self, 
                    name : str,
                    operating_power : float,
                    dt : float,
                    status : str = DISABLED,
                    t : float = 0.0,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of an Abstract Component 

        ### Arguments:
            - name (`str`) : name of the component
            - status (`str`) : initial status of the component
            - t (`float` or `int`) : initial updated time  
            - id (`str`) : identifying number for this component in uuid format
        """
        super().__init__()
                
        self.name = name
        self.operating_power = operating_power
        self.dt = dt
        self.status = status
        self.t = t
        self.id = str(uuid.UUID(id)) if id is not None else str(uuid.uuid1())

    @abstractmethod
    def update(self, **kwargs) -> None:
        """
        Propagates and updates the current state of the component.
        """
        pass

    @abstractmethod
    def perform_action(self, action : ComponentAction, t : Union[int, float]) -> bool:
        """
        Performs an action on this component

        ### Arguments:
            - action (:obj:`ComponentAction`) : action to be performed
            - t (`float` or `int`) : current simulation time in [s]

        ### Returns:
            - boolean value indicating if performing the action was successful or not
        """
        self.t = t

    @abstractmethod
    def is_critical(self, **kwargs) -> bool:
        """
        Returns true if the component is in a critical state
        """
        pass

    @abstractmethod
    def is_failure(self, **kwargs) -> bool:
        """
        Returns true if the component is in a failure state
        """
        pass

    @abstractmethod
    def predict_critical(self, **kwags) -> float:
        """
        Given the current state of the component, this method predicts when a critical state will be reached.

        Returns the time where this will ocurr in simulation seconds.
        """
        pass

    @abstractmethod
    def predict_failure(self, **kwags) -> float:
        """
        Given the current state of the component, this method predicts when a failure state will be reached.

        Returns the time where this will ocurr in simulation seconds.
        """
        pass

    def to_dict(self) -> dict:
        """
        Crates a dictionary containing all information contained in this component object
        """
        return dict(self.__dict__)
    
class Battery(AbstractComponent):
    """
    # Battery Component 

    Represents a battery that is part of the EPS sybstem onboard an agent's Engineering Module

    ### Attributes:
        - max_energy (`float`) : maximum amount of energy a battery can hold
        - conception_energy (`float`) : amount of maximum energy at the conception of a battery object
        - current_energy (`float`) : amount of energy currently available in the battery
        - load (`float`) : the power strain due to the components the battery is powering
    """
    def __init__(   self, 
                    name : str,
                    max_energy : float,
                    dt : float,
                    operating_power : float = 0,
                    status : str = AbstractComponent.DISABLED,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of an Abstract Component 

        ### Arguments:
            - name (`str`) : name of the component
            - status (`str`) : initial status of the component
            - t (`float` or `int`) : initial updated time  
            - id (`str`) : identifying number for this component in uuid format
        """
        super().__init__(name, operating_power, dt)
                
        self.conception_energy = max_energy
        self.max_energy = max_energy
        self.current_energy = 0
        self.load = 0

    def update(self, t : Union[int, float]):
        self.current_energy = self.current_energy - self.load*(t - self.t)
        self.t = t

    def is_critical(self) -> bool:
        if self.current_energy - self.load*self.dt < 0.1*self.max_energy or self.current_energy- self.load*self.dt > 0.9*self.max_energy:
            self.max_energy *= 0.9
            self.status = super().CRITICAL
            return True
        return False
    
    def is_failure(self) -> bool:
        if self.max_energy <= 0.1*self.conception_energy:
            self.status = super().FAILED
            return True
        return False

    def predict_critical(self) -> float:
        if (self.current_energy - 0.1*self.max_energy)/self.load > (self.current_energy - 0.9*self.max_energy)/self.load:
            return (self.current_energy - 0.1*self.max_energy)/self.load
        return (self.current_energy - 0.9*self.max_energy)/self.load


    def predict_failure(self) -> float:
        pass

    def perform_action(self, action : ComponentAction, t : Union[int, float]) -> bool:
        """
        Performs an action on this component

        ### Arguments:
            - action (:obj:`ComponentAction`) : action to be performed
            - t (`float` or `int`) : current simulation time in [s]

        ### Returns:
            - boolean value indicating if performing the action was successful or not
        """
        self.t = t

        if isinstance(action, ComponentProvidePower):
            receiver_power = action.receiver_power
            self.load += receiver_power
            
            #self.status = ENABLED
            return True

        elif isinstance(action, ComponentStopProvidePower):
            self.load -= action.receiver_power
        
        elif isinstance(action, ComponentChargeBattery):
            self.load -= action.charging_power

class SolarPanel(AbstractComponent):
    """
    # Solar Panel Component 

    Represents a solar panel that is part of the EPS sybstem onboard an agent's Engineering Module

    ### Attributes:
        - power (`float`) : maximum amount of power that a solar panel can provide
        - size (`float`) : the area of the solar array in meters squared
        - load (`float`) : the power strain due to the components the solar panel is powering
    """
    ECLIPSE = [(1500,2000),(3500,4000),(6500,7000),(9500,10000)]

    def __init__(   self, 
                    name : str,
                    size : float,
                    area_power : float,
                    dt : float,
                    operating_power : float = 0,
                    status : str = AbstractComponent.DISABLED,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of an Abstract Component 

        ### Arguments:
            - name (`str`) : name of the component
            - size (`str`) : the area of the solar array in meters squared
            - area_power (`str`) : power provided per square meter
            - status (`str`) : initial status of the component
            - t (`float` or `int`) : initial updated time  
            - id (`str`) : identifying number for this component in uuid format
        """
        super().__init__(name, operating_power, dt)
                
        if(self.is_failure()): 
            self.power = 0
        else:
            self.power = size * area_power
            
        self.size = size
        self.area_power = area_power
        self.load = 0

    def update(self, t : Union[int, float]):
        self.t = t

        if(self.is_failure()): 
            self.power = 0
        else:
            self.power = self.size * self.area_power

    def is_critical(self):
        for lapse in self.ECLIPSE:
            if self.t+self.dt >= lapse[0] and self.t<=lapse[1]:
                self.status = super().CRITICAL
                return True
    
    def is_failure(self):
        for lapse in self.ECLIPSE:
            if self.t>=lapse[0] and self.t<=lapse[1]:
                self.status = super().FAILED
                return True

    def predict_critical(self):
        time_to_crit = 0
        for lapse in self.ECLIPSE:
            if abs(lapse[0]-self.t-self.dt) < time_to_crit or time_to_crit == 0:
                time_to_crit = abs(lapse[0]-self.t-self.dt)
        return time_to_crit

    def predict_failure(self):
        time_to_fail = 0
        for lapse in self.ECLIPSE:
            if abs(lapse[0]-self.t) < time_to_fail or time_to_fail == 0:
                time_to_fail = abs(lapse[0]-self.t)
        return time_to_fail

    def perform_action(self, action : ComponentAction, t : Union[int, float]) -> bool:
        """
        Performs an action on this component

        ### Arguments:
            - action (:obj:`ComponentAction`) : action to be performed
            - t (`float` or `int`) : current simulation time in [s]

        ### Returns:
            - boolean value indicating if performing the action was successful or not
        """
        self.t = t

        if isinstance(action, ComponentProvidePower):
            self.load += action.receiver_power
            self.status = super().ENABLED

        elif isinstance(action, ComponentStopProvidePower):
            self.load -= action.receiver_power
        
        elif isinstance(action, ComponentChargeBattery):
            self.load += action.charging_power


class Instrument(AbstractComponent):
    """
    # Instrument Component 

    Represents an instrument that is connected to a battery in the EPS sybstem onboard an agent's Engineering Module

    ### Attributes:
        - 
    """
    def __init__(   self, 
                    name : str,
                    operating_power: float,
                    dt : float,
                    t: float = 0,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of an Abstract Component 

        ### Arguments:
            - name (`str`) : name of the component
            - status (`str`) : initial status of the component
            - t (`float` or `int`) : initial updated time  
            - id (`str`) : identifying number for this component in uuid format
        """
        super().__init__(name, operating_power, dt)

    def update(self, t : Union[int, float]):
        self.t = t

    def is_critical(self):
        pass
    
    def is_failure(self):
        pass

    def predict_critical(self):
        pass

    def predict_failure(self):
        pass

    def perform_action(self, action : ComponentAction, t : Union[int, float]) -> bool:
        """
        Performs an action on this component

        ### Arguments:
            - action (:obj:`ComponentAction`) : action to be performed
            - t (`float` or `int`) : current simulation time in [s]

        ### Returns:
            - boolean value indicating if performing the action was successful or not
        """
        self.t = t

        if isinstance(action, ):
            pass