from abc import ABC, abstractmethod
from typing import Union
import uuid
from nodes.engineering.actions import SubsystemAction, SubsystemProvidePower, SubsystemStopProvidePower, ComponentProvidePower, ComponentStopProvidePower, ComponentChargeBattery
from nodes.engineering.components import AbstractComponent, SolarPanel, Battery


class AbstractSubsystem(ABC):
    """
    Represents a subsystem onboard an agent's Engineering Module
    
    ### Attributes:
        - name (`str`) : name of the subsystem
        - status (`str`) : current status of the subsystem
        - t (`float` or `int`) : last updated time
        - id (`str`) : identifying number for this subsystem in uuid format
    """
    ENABLED = 'ENABLED'
    DISABLED = 'DISABLED'
    CRITICAL = 'CRITICAL'
    FAILED = 'FAILED'

    def __init__(   self, 
                    name : str,
                    components : list,
                    status : str = DISABLED,
                    t : float = 0.0,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of an Abstract Subsystem 

        ### Arguments:
            - name (`str`) : name of the subsystem
            - components (`list`): list of components comprising this subsystem
            - status (`str`) : initial status of the subsystem
            - t (`float` or `int`) : initial updated time  
            - id (`str`) : identifying number for this task in uuid format
        """
        super().__init__()
                
        # check parameters
        if not isinstance(components, list):
            raise ValueError(f'`components` must be of type `list`. is of type {type(components)}.')
        for component in components:
            if not isinstance(component, AbstractComponent):
                raise ValueError(f'elements of list `components` must be of type `Component`. contains element of type {type(component)}.')
        
        # assign values
        self.name = name
        self.status = status
        self.t = t
        self.id = str(uuid.UUID(id)) if id is not None else str(uuid.uuid1())

        # converts the list of component objects into a dictionary with component names as the keys and component objects as values
        dictionary = {}
        for component in components:
            component : AbstractComponent
            dictionary[component.name] = component
        self.components = dictionary

    @abstractmethod
    def update(self, **kwargs) -> None:
        """
        Propagates and updates the current state of the subsystem.
        """
        pass

    @abstractmethod
    def perform_action(self, action : SubsystemAction, t : Union[int, float]) -> bool:
        """
        Performs an action on this subsystem

        ### Arguments:
            - action (:obj:`SubsystemAction`) : action to be performed
            - t (`float` or `int`) : current simulation time in [s]

        ### Returns:
            - boolean value indicating if performing the action was successful or not
        """
        self.t = t

    @abstractmethod
    def is_critical(self, **kwargs) -> bool:
        """
        Returns true if the subsystem is in a critical state
        """
        pass

    @abstractmethod
    def is_failure(self, **kwargs) -> bool:
        """
        Returns true if the subsystem is in a failure state
        """
        pass

    @abstractmethod
    def predict_critical(self, **kwags) -> float:
        """
        Given the current state of the subsystem, this method predicts when a critical state will be reached.

        Returns the time where this will ocurr in simulation seconds.
        """
        pass

    @abstractmethod
    def predict_failure(self, **kwags) -> float:
        """
        Given the current state of the subsystem, this method predicts when a failure state will be reached.

        Returns the time where this will ocurr in simulation seconds.
        """
        pass

    def to_dict(self) -> dict:
        """
        Crates a dictionary containing all information contained in this agent state object
        """
        return dict(self.__dict__)
    
    @abstractmethod
    def decompose_instructions(self, **kwags) -> float:
        ""

        ""
        pass

class EPSubsystem(AbstractSubsystem):
    """
    Represents the EPS subsystem onboard an agent's Engineering Module
    """
    def __init__(   self, 
                    components : list,
                    payload : list,
                    status : str = AbstractSubsystem.ENABLED,
                    t : float = 0.0,
                    connections : dict = None,
                    id : str = None
                    ) -> None:
        """
        Initiates an instance of the EPS Subsystem 

        ### Arguments:
            - components (`list`): list of components comprising this subsystem
            - status (`str`) : initial status of the subsystem
            - t (`float` or `int`) : initial simulation time  
            - connections (`dict`): maps out the possible components to provide and (or) be provided power
            - id (`str`) : identifying number for this task in uuid format
        """
        super().__init__("EPS", components, status, t, id)

        self.connections = connections

        # check parameters
        if not isinstance(components, list):
            raise ValueError(f'`components` must be of type `list`. is of type {type(components)}.')
        for component in components:
            if not isinstance(component, AbstractComponent):
                raise ValueError(f'elements of list `components` must be of type `Component`. contains element of type {type(component)}.')
            elif not isinstance(component, Battery) and not isinstance(component, SolarPanel): 
                raise NotImplementedError(f'component of type {type(component)} not yet supported by EPS subsystem.')
        
        # construct connections
        if not connections:
            batteries = [component for component in components if isinstance(component, Battery)]
            solarpanels = [component for component in components if isinstance(component, SolarPanel)]

            self.connections = {}
            for battery in batteries:
                self.connections[battery] = [(instrument, 0) for instrument in payload]
            
            for solarpanel in solarpanels:
                connection = []
                for instrument in payload:
                    connection.append( (instrument, 0) )

                for battery in batteries:
                    connection.append( (battery, 0) )

                connections[solarpanel] = connection
        
            
    def update(self, t):
        """
        Updates all components in the components list
        """

        # update time
        self.t = t

        # update component states
        components = list(self.components.values())
        for component in components:
            component : AbstractComponent
            component.update(t)
        
        sources = list(self.connections.keys())
        for source in sources:
            if isinstance(source, SolarPanel):
                if source.is_failure():
                    for broken_connection in self.connections[source]:
                        if isinstance(broken_connection[0],Battery) and broken_connection[1] > 0:
                            component_action = ComponentChargeBattery(-broken_connection[1], t)
                            source.perform_action(component_action, t)
                            broken_connection[0].perform_action(component_action, t)
                            self.update_connections(source, broken_connection[0], 0)
                        elif broken_connection[1] > 0:
                            component_action = ComponentStopProvidePower(broken_connection[1], t)
                            source.perform_action(component_action, t)
                            self.update_connections(source, broken_connection[0], 0)

                            subsystem_action = SubsystemProvidePower(broken_connection[0].name, t)
                            self.perform_action(subsystem_action, t)

                else:
                    charging_power = source.power - source.load
                    battery_count = 0
                    for connection in self.connections[source]:
                        if isinstance(connection[0], Battery) and connection[0].current_energy <= 0.9*connection[0].max_energy:
                            battery_count += 1

                    for connection in self.connections[source]:
                        if isinstance(connection[0], Battery):
                            if connection[0].current_energy <= 0.9*connection[0].max_energy:
                                component_action = ComponentChargeBattery(charging_power/battery_count, t)
                                source.perform_action(component_action, t)
                                connection[0].perform_action(component_action, t)
                                self.update_connections(source, connection[0], connection[1]+charging_power/battery_count)
                            elif connection[0].current_energy >= 0.9*connection[0].max_energy and connection[1] > 0:
                                component_action = ComponentChargeBattery(-connection[1], t)
                                source.perform_action(component_action, t)
                                connection[0].perform_action(component_action, t)
                                self.update_connections(source, connection[0], 0)

            elif isinstance(source, Battery) and source.energy_stored < 0.1*source.energy_capacity:
                for broken_connection in self.connections[source]:
                    if broken_connection[1] > 0:
                        component_action = ComponentStopProvidePower(broken_connection[1], t)
                        source.perform_action(component_action, t)
                        self.update_connections(source, broken_connection[0], 0)

                        subsystem_action = SubsystemProvidePower(broken_connection[0].name, t)
                        self.perform_action(subsystem_action, t)

    def perform_action(self, action : SubsystemAction, t : Union[int, float]) -> bool:
        self.t = t
        
        if isinstance(action, SubsystemProvidePower):
            receiver = self.components[action.receiver]
            receiver_power = receiver.operating_power

            ## Check if component is a connection of each source ##
            possible_sources = []
            values = list(self.connections.values())
            for i in range(len(values)):
                components = values[i]
                for component in components:
                    if component[0] == receiver:
                        possible_sources.append(list(self.connections.keys())[i])

            ## use only solar panel source if possible and if not add all other batteries ##
            chosen_sources = []
            for source in possible_sources:
                if isinstance(source, SolarPanel) and not source.is_failure():
                    chosen_sources.insert(0, source)
                    if source.power - source.load > receiver_power:
                        chosen_sources = [source]
                        break
                elif isinstance(source, Battery) and source.energy_stored > 0.1*source.energy_capacity:
                    chosen_sources.append(source)
            
            ## return False if no source can be found to power the receiver ##
            if(len(chosen_sources) == 0):
                self.status = super().FAILED
                return False
            
            solar_power = 0
            battery_power = 0
            for chosen_source in chosen_sources:
                ## distribute the power between chosen sources and update connections dictionary #
                if isinstance(chosen_source, SolarPanel):
                    ## stop the power to charge batteries to prioritize powering component ##
                    ## extra power will begin charging batteries in the next dt ##
                    for broken_connection in self.connections[source]:
                        if isinstance(broken_connection[0],Battery):
                            component_action = ComponentChargeBattery(-broken_connection[1], t)
                            source.perform_action(component_action, t)
                            broken_connection[0].perform_action(component_action, t)
                            self.update_connections(source, broken_connection[0], 0)
                    if chosen_source.power - chosen_source.load > receiver_power:
                        solar_power = receiver_power
                        component_action = ComponentProvidePower(solar_power, self.t)
                        chosen_source.perform_action(component_action, self.t)
                        self.update_connections(chosen_source, receiver, solar_power)
                        break
                    else:
                        solar_power = chosen_source.power - chosen_source.load
                        component_action = ComponentProvidePower(solar_power, self.t)
                        chosen_source.perform_action(component_action, self.t)
                        self.update_connections(chosen_source, receiver, solar_power)

                elif isinstance(chosen_source, Battery):
                    if solar_power == 0:
                        battery_power = (receiver_power)/(len(chosen_sources))    
                    else:
                        battery_power = (receiver_power-solar_power)/(len(chosen_sources)-1)
                    component_action = ComponentProvidePower(battery_power, self.t)
                    chosen_source.perform_action(component_action, self.t)

                    self.update_connections(chosen_source, receiver, battery_power)

            self.status = super().ENABLED
            return True

        elif isinstance(action, SubsystemStopProvidePower):
            receiver = self.components[action.receiver]
            powering_sources = []

            ## Find which sources power a component and create ComponentStopProvidePower##
            values = list(self.connections.values())
            for i in range(len(values)):
                components = values[i]
                for component in components:
                    if component[0] == receiver and component[1] > 0:
                        powering_source = list(self.connections.keys())[i]
                        component_action = ComponentStopProvidePower(component[1], self.t)
                        powering_source.perform_action(component_action, self.t)
                        self.update_connections(powering_source, receiver, 0)

            return True
                
        elif isinstance():
            pass

    def is_failure(self):
        """
        Returns true if the subsystem is in a failure state
        Is defined by a source component such as a battery or solar array being in a falure state
        """
        count = 0
        sources = list(self.components.keys())
        for source in sources:
            if (source.isinstance(SolarPanel) and source.is_failure()) or (source.isinstance(Battery) and source.is_critical()):
                count +=1
        if count == len(sources):
            self.status = super().FAILED
            return True
        return False

    def is_critical(self):
        """
        Returns true if the subsystem is in a critical state

        Is defined by a source component such as a battery or solar array being in a critical state
        """
        sources = list(self.components.keys())
        for source in sources:
            if source.is_critical():
                self.status = super().CRITICAL
                return True
        return False
    
    def predict_failure(self):
        """
        Given the current state of the subsystem, this method predicts when a failure state will be reached.

        Is defined by the source components with the shortest predicted failure state

        Returns the time where this will ocurr in simulation seconds.
        """
        time_to_fail = 0
        sources = list(self.components.keys())
        for source in sources:
            component_failure = source.predict_failure()
            if component_failure > time_to_fail:
                time_to_fail = component_failure

            depletion = -source.current_energy/source.load
            if depletion > time_to_fail:
                time_to_fail = depletion

        return time_to_fail

    def predict_critical(self) -> float:
        """
        Given the current state of the subsystem, this method predicts when a critical state will be reached.

        Is defined by the source components with the shortest predicted critical state

        Returns the time where this will ocurr in simulation seconds.
        """
        time_to_crit = 0
        components = list(self.components.values())
        for component in components:
            temp = component.predict_critical()
            if temp < time_to_crit:# and dict(component) == "source":
                time_to_crit = temp
        return time_to_crit
    

    def decompose_instructions(self):
        pass

    def update_connections(self, source, receiver, power):
        value = self.connections[source]
        updated_values = []
        for component in value:
            if component[0] == receiver:
                updated_values.append((component[0],power))
            else:
                updated_values.append(component)
        self.connections[source] = updated_values