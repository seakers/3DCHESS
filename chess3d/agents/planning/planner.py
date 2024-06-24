
import math
import queue

from instrupy.base import Instrument, BasicSensorModel
from instrupy.util import ViewGeometry, SphericalGeometry
from orbitpy.util import Spacecraft

from dmas.modules import *
from dmas.utils import runtime_tracker

from chess3d.agents.planning.plan import Plan, Preplan
from chess3d.agents.orbitdata import OrbitData, TimeInterval
from chess3d.agents.states import *
from chess3d.agents.science.requests import *
from chess3d.messages import *

class AbstractPlanner(ABC):
    """ 
    Describes a generic planner that, given a new set of percepts, decides whether to generate a new plan
    """
    def __init__(self, 
                 logger : logging.Logger = None) -> None:
        # initialize object
        super().__init__()

        # check inputs
        if not isinstance(logger,logging.Logger) and logger is not None: 
            raise ValueError(f'`logger` must be of type `Logger`. Is of type `{type(logger)}`.')

        # initialize attributes
        self.known_reqs = set()                 # set of known measurement requests
        self.pending_reqs_to_broadcast = set()  # set of observation requests that have not been broadcasted
        self.pending_relays = set()             # set of relay messages to be broadcasted
        self.completed_broadcasts = set()       # set of completed broadcasts
        self.stats = {}                         # collector for runtime performance statistics
        
        # set attribute parameters
        self._logger = logger               # logger for debugging

    @abstractmethod
    def update_percepts( self,
                         state : SimulationAgentState,
                         incoming_reqs : list,
                         relay_messages : list,
                         completed_actions : list,
                         **kwargs
                        ) -> None:
        """ Updates internal knowledge based on incoming percepts """
        
        # check parameters
        for req in incoming_reqs:                   assert isinstance(req, MeasurementRequest)
        for relay_message in relay_messages:        assert isinstance(relay_message, SimulationMessage)
        for completed_action in completed_actions:  assert isinstance(completed_action, AgentAction)

        # update list of known requests
        self.known_reqs.update(incoming_reqs)

        # update list of completed broadcasts
        completed_broadcasts = [message_from_dict(**action.msg) 
                                for action in completed_actions
                                if isinstance(action, BroadcastMessageAction)]
        self.completed_broadcasts.update(completed_broadcasts)
        assert all([isinstance(msg, SimulationMessage) for msg in self.completed_broadcasts])
        
        # update list of pending relays 
        self.pending_relays.update(relay_messages)
        for completed_broadcast in completed_broadcasts:
            if completed_broadcast in self.pending_relays:
                self.pending_relays.remove(completed_broadcast)

        # get list of measurement requests newly generated by the parent agent
        my_reqs = {req 
                   for req in incoming_reqs
                   if isinstance(req, MeasurementRequest)
                   and req.requester == state.agent_name}

        # update list of pending requests to broadcast
        self.pending_reqs_to_broadcast.update(my_reqs)

        # remove from list of pending requests to breadcasts if they've been broadcasted already
        broadcasted_reqs = {MeasurementRequest.from_dict(msg.req) 
                            for msg in self.completed_broadcasts
                            if isinstance(msg, MeasurementRequestMessage)}
        for req in broadcasted_reqs:
            if req in self.pending_reqs_to_broadcast:
                self.pending_reqs_to_broadcast.remove(req)


    @abstractmethod
    def needs_planning(self, **kwargs) -> bool:
        """ Determines whether planning is triggered """ 
        
    @abstractmethod
    def generate_plan(self, **kwargs) -> Plan:
        """ Creates a plan for the agent to perform """

    def _schedule_broadcasts(self, 
                             state : SimulationAgentState, 
                             orbitdata : OrbitData
                            ) -> list:
        """ 
        Schedules any broadcasts to be done. 
        
        By default it schedules the relaying of any incoming relay messages
        """
        if not isinstance(state, SatelliteAgentState):
            raise NotImplementedError(f'Broadcast scheduling for agents of type `{type(state)}` not yet implemented.')
        elif orbitdata is None:
            raise ValueError(f'`orbitdata` required for agents of type `{type(state)}`.')

        # initialize list of broadcasts to be done
        broadcasts = []       

        # schedule generated measurement request broadcasts
        if self.pending_reqs_to_broadcast:
            # find best path for broadcasts
            path, t_start = self._create_broadcast_path(state, orbitdata)

            # check feasibility of path found
            if t_start >= 0:
                # create a broadcast action for all unbroadcasted measurement requests
                while self.pending_reqs_to_broadcast:
                    # get next request to broadcast
                    req : MeasurementRequest = self.pending_reqs_to_broadcast.pop()

                    # create broadcast action
                    msg = MeasurementRequestMessage(state.agent_name, state.agent_name, req.to_dict(), path=path)
                    broadcast_action = BroadcastMessageAction(msg.to_dict(), t_start)
                    
                    broadcasts.append(broadcast_action)

        # schedule message relay
        relay_broadcasts = [self._schedule_relay(relay) for relay in self.pending_relays]
        broadcasts.extend(relay_broadcasts)    
                        
        # return scheduled broadcasts
        return broadcasts 
    
    def _create_broadcast_path(self, 
                               state : SimulationAgentState, 
                               orbitdata : OrbitData = None,
                               t_init : float = None
                               ) -> tuple:
        """ Finds the best path for broadcasting a message to all agents using depth-first-search 
        
        ### Arguments:
            - state (`SimulationAgentState`): current state of the agent
            - orbitdata (`OrbitData`): coverage data of agent if it is of type `SatelliteAgent`
            - t_init (`float`): ealiest desired broadcast time
        """
        if not isinstance(state, SatelliteAgentState):
            raise NotImplementedError(f'Broadcast routing path not yet supported for agents of type `{type(state)}`')
        
        # get earliest desired broadcast time 
        t_init = state.t if t_init is None or t_init < state.t else t_init

        # populate list of all agents except the parent agent
        target_agents = [target_agent 
                         for target_agent in orbitdata.isl_data 
                         if target_agent != state.agent_name]
        
        if not target_agents: 
            # no other agents in the simulation; no need for relays
            return ([], t_init)
        
        # check if broadcast needs to be routed
        earliest_accesses = [   orbitdata.get_next_agent_access(target_agent, t_init) 
                                for target_agent in target_agents]           
        
        same_access_start = [   abs(access.start - earliest_accesses[0].start) < 1e-3
                                for access in earliest_accesses 
                                if isinstance(access, TimeInterval)]
        same_access_end = [     abs(access.end - earliest_accesses[0].end) < 1e-3
                                for access in earliest_accesses 
                                if isinstance(access, TimeInterval)]

        if all(same_access_start) and all(same_access_end):
            # all agents are accessing eachother at the same time; no need for mesasge relays
            return ([], t_init)   

        # look for relay path using depth-first search

        # initialize queue
        q = queue.Queue()
        
        # initialize min path and min path cost
        min_path = []
        min_times = []
        min_cost = np.Inf

        # add parent agent as the root node
        q.put((state.agent_name, [], [], 0.0))

        while not q.empty():
            # get next node in the search
            _, current_path, current_times, path_cost = q.get()

            # check if path is complete
            if len(target_agents) == len(current_path):
                # check if minimum cost
                if path_cost < min_cost:
                    min_cost = path_cost
                    min_path = [path_element for path_element in current_path]
                    min_times = [path_time for path_time in current_times]

            # add children nodes to queue
            for receiver_agent in [receiver_agent for receiver_agent in target_agents 
                                    if receiver_agent not in current_path
                                    and receiver_agent != state.agent_name
                                    ]:
                # query next access interval to children nodes
                t_access : float = state.t + path_cost

                access_interval : TimeInterval = orbitdata.get_next_agent_access(receiver_agent, t_access)
                
                if access_interval.start < np.Inf:
                    new_path = [path_element for path_element in current_path]
                    new_path.append(receiver_agent)

                    new_cost = access_interval.start - state.t

                    new_times = [path_time for path_time in current_times]
                    new_times.append(new_cost + state.t)

                    q.put((receiver_agent, new_path, new_times, new_cost))

        # check if earliest broadcast time is valid
        if min_times: assert state.t <= min_times[0]

        # return path and broadcast start time
        return (min_path, min_times[0]) if min_path else ([], np.Inf)
    
    @runtime_tracker
    def _schedule_relay(self, relay_message : SimulationMessage) -> list:
        raise NotImplementedError('Relay scheduling not yet supported.')
        
        # check if relay message has a valid relay path
        assert relay.path

        # find next destination and access time
        next_dst = relay.path.pop(0)
        
        # query next access interval to children nodes
        sender_orbitdata : OrbitData = orbitdata[state.agent_name]
        access_interval : TimeInterval = sender_orbitdata.get_next_agent_access(next_dst, state.t)
        t_start : float = access_interval.start

        if t_start < np.Inf:
            # if found, create broadcast action
            broadcast_action = BroadcastMessageAction(relay.to_dict(), t_start)
            
            # check broadcast start; only add to plan if it's within the planning horizon
            if t_start <= state.t + self.horizon:
                broadcasts.append(broadcast_action)

    @runtime_tracker
    def _schedule_maneuvers(    self, 
                                state : SimulationAgentState, 
                                specs : object,
                                observations : list,
                                clock_config : ClockConfig,
                                orbitdata : OrbitData = None
                            ) -> list:
        """
        Generates a list of AgentActions from the current path.

        Agents look to move to their designated measurement target and perform the measurement.

        ## Arguments:
            - state (:obj:`SimulationAgentState`): state of the agent at the start of the path
            - specs (`dict` or `Sapcecraft`): contains information regarding the physical specifications of the agent
            - path (`list`): list of tuples indicating the sequence of observations to be performed and time of observation
            - t_init (`float`): start time for plan
            - clock_config (:obj:`ClockConfig`): clock being used for this simulation
        """

        if not isinstance(state, SatelliteAgentState):
            raise NotImplementedError(f'Maneuver scheduling for agents of type `{type(state)}` not yet implemented.')
        elif not isinstance(specs, Spacecraft):
            raise ValueError(f'`specs` needs to be of type `Spacecraft` for agents of state type `{type(state)}`. Is of type `{type(specs)}`.')
        elif orbitdata is None:
            raise ValueError(f'`orbitdata` required for agents of type `{type(state)}`.')

        # compile instrument field of view specifications   
        cross_track_fovs = self.collect_fov_specs(specs)

        # get pointing agility specifications
        adcs_specs : dict = specs.spacecraftBus.components.get('adcs', None)
        if adcs_specs is None: raise ValueError('ADCS component specifications missing from agent specs object.')

        max_slew_rate = float(adcs_specs['maxRate']) if adcs_specs.get('maxRate', None) is not None else None
        if max_slew_rate is None: raise ValueError('ADCS `maxRate` specification missing from agent specs object.')

        max_torque = float(adcs_specs['maxTorque']) if adcs_specs.get('maxTorque', None) is not None else None
        if max_torque is None: raise ValueError('ADCS `maxTorque` specification missing from agent specs object.')

        # initialize maneuver list
        maneuvers = []

        for i in range(len(observations)):
            action_sequence_i = []

            curr_observation : ObservationAction = observations[i]
            t_img = curr_observation.t_start
                        
            # estimate previous state
            if i == 0:
                t_prev = state.t
                prev_state : SatelliteAgentState = state.copy()
                
            else:
                prev_observation : ObservationAction = observations[i-1]
                t_prev = prev_observation.t_end if prev_observation is not None else state.t
                prev_state : SatelliteAgentState = state.propagate(t_prev)          

                prev_maneuvers = [maneuver for maneuver in maneuvers
                                  if isinstance(maneuver, ManeuverAction)]
                
                if prev_maneuvers:
                    prev_maneuvers.sort(key=lambda a: a.t_start)
                    last_maneuver : ManeuverAction = prev_maneuvers[-1]
                    prev_state.attitude = [th for th in last_maneuver.final_attitude]
                # prev_state.attitude = [prev_observation.look_angle, 0.0, 0.0]      

            # maneuver to point to target
            t_maneuver_end = None
            if isinstance(state, SatelliteAgentState):
                prev_state : SatelliteAgentState
                
                dth = abs(curr_observation.look_angle - prev_state.attitude[0])
                cross_track_fov = cross_track_fovs[curr_observation.instrument_name]

                if dth <= cross_track_fov / 2.0:
                    # no need to maneuver
                    th_f = prev_state.attitude[0]
                else:
                    # maneuver needed
                    th_f = curr_observation.look_angle

                    # TODO make efficient maneuvers
                    # th_f_sign = abs(curr_observation.look_angle)/curr_observation.look_angle
                    # th_f = abs(curr_observation.look_angle)- cross_track_fov / 2.0
                    # th_f *= th_f_sign

                dt = abs(th_f - prev_state.attitude[0]) / max_slew_rate
                    
                t_maneuver_start = prev_state.t
                t_maneuver_end = t_maneuver_start + dt

                if abs(t_maneuver_start - t_maneuver_end) >= 1e-3:
                    action_sequence_i.append(ManeuverAction([th_f, 0, 0], 
                                                            t_maneuver_start, 
                                                            t_maneuver_end))   
                else:
                    t_maneuver_end = None

            # move to target
            t_move_start = t_prev if t_maneuver_end is None else t_maneuver_end
            t_move_end = t_img
            future_state : SatelliteAgentState = state.propagate(t_move_end)
            final_pos = future_state.pos
            
            # quantize travel maneuver times if needed
            if isinstance(clock_config, FixedTimesStepClockConfig):
                dt = clock_config.dt
                if t_move_start < np.Inf:
                    t_move_start = dt * math.floor(t_move_start/dt)
                if t_move_end < np.Inf:
                    t_move_end = dt * math.ceil(t_move_end/dt)
            
            # add travel maneuver if required
            if abs(t_move_start - t_move_end) >= 1e-3:
                
                if t_move_end < t_move_start:
                    x = 1

                move_action = TravelAction(final_pos, t_move_start, t_move_end)
                action_sequence_i.append(move_action)
            
            # wait for measurement action to start
            if t_move_end < t_img:
                action_sequence_i.append( WaitForMessages(t_move_end, t_img) )

            maneuvers.extend(action_sequence_i)

        return maneuvers
    
    def collect_fov_specs(self, specs : Spacecraft) -> dict:
        # compile instrument field of view specifications   
        cross_track_fovs = {instrument.name: np.NAN for instrument in specs.instrument}
        for instrument in specs.instrument:
            cross_track_fov = []
            for instrument_model in instrument.mode:
                if isinstance(instrument_model, BasicSensorModel):
                    instrument_fov : ViewGeometry = instrument_model.get_field_of_view()
                    instrument_fov_geometry : SphericalGeometry = instrument_fov.sph_geom
                    if instrument_fov_geometry.shape == 'RECTANGULAR':
                        cross_track_fov.append(instrument_fov_geometry.angle_width)
                    else:
                        raise NotImplementedError(f'Extraction of FOV for instruments with view geometry of shape `{instrument_fov_geometry.shape}` not yet implemented.')
                else:
                    raise NotImplementedError(f'measurement data query not yet suported for sensor models of type {type(instrument_model)}.')
            cross_track_fovs[instrument.name] = max(cross_track_fov)

        return cross_track_fovs
        
    @runtime_tracker
    def is_observation_path_valid(self, 
                                  state : SimulationAgentState, 
                                  specs : object,
                                  observations : list
                                  ) -> bool:
        """ Checks if a given sequence of observations can be performed by a given agent """
        
        if isinstance(state, SatelliteAgentState) and isinstance(specs, Spacecraft):

            # get pointing agility specifications
            adcs_specs : dict = specs.spacecraftBus.components.get('adcs', None)
            if adcs_specs is None: raise ValueError('ADCS component specifications missing from agent specs object.')

            max_slew_rate = float(adcs_specs['maxRate']) if adcs_specs.get('maxRate', None) is not None else None
            if max_slew_rate is None: raise ValueError('ADCS `maxRate` specification missing from agent specs object.')

            max_torque = float(adcs_specs['maxTorque']) if adcs_specs.get('maxTorque', None) is not None else None
            if max_torque is None: raise ValueError('ADCS `maxTorque` specification missing from agent specs object.')

            # compile instrument field of view specifications   
            cross_track_fovs : dict = self.collect_fov_specs(specs)

            # check if every observation can be reached from the prior measurement
            for j in range(len(observations)):
                i = j - 1

                # check if there was an observation performed previously
                if i >= 0: # there was a prior observation performed

                    # estimate the state of the agent at the prior mesurement
                    observation_i : ObservationAction = observations[i]
                    th_i = observation_i.look_angle
                    t_i = observation_i.t_end

                else: # there was prior measurement

                    # use agent's current state as previous state
                    th_i = state.attitude[0]
                    t_i = state.t

                # estimate the state of the agent at the given measurement
                observation_j : ObservationAction = observations[j]
                th_j = observation_j.look_angle
                t_j = observation_j.t_start

                # check if desired instrument is contained within the satellite's specifications
                if observation_j.instrument_name not in [instrument.name for instrument in specs.instrument]:
                    return False 
                
                assert th_j != np.NAN and th_i != np.NAN # TODO: add case where the target is not visible by the agent at the desired time according to the precalculated orbitdata

                # estimate maneuver time betweem states
                fov = cross_track_fovs[observation_j.instrument_name]
                if abs(th_j - th_i) <= fov / 2.0:
                    dt_maneuver = 0.0
                else:
                    dt_maneuver = abs(th_j - th_i) / max_slew_rate

                dt_measurements = t_j - t_i

                assert dt_measurements >= 0.0 and dt_maneuver >= 0.0

                # Slewing constraint:
                # check if there's enough time to maneuver from one observation to another
                if dt_maneuver - dt_measurements >= 1e-9:
                    # there is not enough time to maneuver; flag current observation plan as unfeasible for rescheduling
                    return False
                
                # Torque constraint:
                # TODO check if the agent has enough torque in its reaction wheels to perform the maneuver
                            
            # if all measurements passed the check; observation path is valid
            return True
        else:
            raise NotImplementedError(f'Observation path validity check for agents with state type {type(state)} not yet implemented.')
        
    # def _print_observation_sequence(self, 
    #                                 state : SatelliteAgentState, 
    #                                 path : list, 
    #                                 orbitdata : OrbitData = None
    #                                 ) -> None :
    #     """ Debugging tool. Prints current observation sequence being considered. """

    #     if not isinstance(state, SatelliteAgentState):
    #         raise NotImplementedError('Observation sequence printouts for non-satellite agents not yet supported.')
    #     elif orbitdata is None:
    #         raise ValueError(f'`orbitdata` required for agents of type `{type(state)}`.')

    #     out = f'\n{state.agent_name}:\n\n\ntarget\tinstr\tt_img\tth\tdt_mmt\tdt_mvr\tValid?\n'

    #     out_temp = [f"N\A       ",
    #                 f"N\A",
    #                 f"\t{np.round(state.t,3)}",
    #                 f"\t{np.round(state.attitude[0],3)}",
    #                 f"\t-",
    #                 f"\t-",
    #                 f"\t-",
    #                 f"\n"
    #                 ]
    #     out += ''.join(out_temp)

    #     for i in range(len(path)):
    #         if i > 0:
    #             measurement_prev : ObservationAction = path[i-1]
    #             t_prev = measurement_i.t_end
    #             lat,lon,_ = measurement_prev.target
    #             obs_prev = orbitdata.get_groundpoint_access_data(lat, lon, measurement_prev.instrument_name, t_prev)
    #             th_prev = obs_prev['look angle [deg]']
    #         else:
    #             t_prev = state.t
    #             th_prev = state.attitude[0]

    #         measurement_i : ObservationAction = path[i]
    #         t_i = measurement_i.t_start
    #         lat,lon,alt = measurement_i.target
    #         obs_i = orbitdata.get_groundpoint_access_data(lat, lon, measurement_i.instrument_name, t_i)
    #         th_i = obs_i['look angle [deg]']

    #         dt_maneuver = abs(th_i - th_prev) / state.max_slew_rate
    #         dt_measurements = t_i - t_prev

    #         out_temp = [f"({round(lat,3)}, {round(lon,3)}, {round(alt,3)})",
    #                         f"  {measurement_i.instrument_name}",
    #                         f"\t{np.round(measurement_i.t_start,3)}",
    #                         f"\t{np.round(th_i,3)}",
    #                         f"\t{np.round(dt_measurements,3)}",
    #                         f"\t{np.round(dt_maneuver,3)}",
    #                         f"\t{dt_maneuver <= dt_measurements}",
    #                         f"\n"
    #                         ]
    #         out += ''.join(out_temp)
    #     out += f'\nn measurements: {len(path)}\n'

    #     print(out)

class AbstractPreplanner(AbstractPlanner):
    """
    # Preplanner

    Conducts operations planning for an agent at the beginning of a planning period. 
    """
    def __init__(   self, 
                    horizon : float = np.Inf,
                    period : float = np.Inf,
                    logger: logging.Logger = None
                ) -> None:
        """
        ## Preplanner 
        
        Creates an instance of a preplanner class object.

        #### Arguments:
            - horizon (`float`) : planning horizon in seconds [s]
            - period (`float`) : period of replanning in seconds [s]
            - logger (`logging.Logger`) : debugging logger
        """
        # initialize planner
        super().__init__(logger)    

        # set parameters
        self.horizon = horizon                               # planning horizon
        self.period = period                                 # replanning period         
        self.plan = Preplan(t=-1,horizon=horizon,t_next=0.0) # initialized empty plan
        
    @runtime_tracker
    def update_percepts(self, 
                        state : SimulationAgentState,
                        current_plan : Plan,
                        incoming_reqs: list, 
                        relay_messages: list, 
                        misc_messages : list,
                        completed_actions: list,
                        aborted_actions : list,
                        pending_actions : list
                        ) -> None:
        
        super().update_percepts(state, incoming_reqs, relay_messages, completed_actions)
    
    @runtime_tracker
    def needs_planning( self, 
                        state : SimulationAgentState,
                        __ : object,
                        current_plan : Plan
                        ) -> bool:
        """ Determines whether a new plan needs to be initalized """    

        return (current_plan.t < 0                  # simulation just started
                or state.t >= self.plan.t_next)     # periodic planning period has been reached

    @runtime_tracker
    def generate_plan(  self, 
                        state : SimulationAgentState,
                        specs : object,
                        clock_config : ClockConfig,
                        orbitdata : OrbitData,
                    ) -> Plan:
        
        # schedule observations
        observations : list = self._schedule_observations(state, specs, clock_config, orbitdata)
        assert self.is_observation_path_valid(state, specs, observations)

        # schedule broadcasts to be perfomed
        broadcasts : list = self._schedule_broadcasts(state, observations, orbitdata)

        # generate maneuver and travel actions from measurements
        maneuvers : list = self._schedule_maneuvers(state, specs, observations, clock_config, orbitdata)
        
        # wait for next planning period to start
        replan : list = self.__schedule_periodic_replan(state, observations, maneuvers)

        # generate plan from actions
        self.plan : Preplan = Preplan(observations, maneuvers, broadcasts, replan, t=state.t, horizon=self.horizon, t_next=state.t+self.period)    

        # return plan and save local copy
        return self.plan.copy()
        
    @abstractmethod
    def _schedule_observations(self, state : SimulationAgentState, specs : object, clock_config : ClockConfig, orbitdata : OrbitData = None) -> list:
        """ Creates a list of observation actions to be performed by the agent """    

    @abstractmethod
    def _schedule_broadcasts(self, state: SimulationAgentState, observations : list, orbitdata: OrbitData) -> list:
        return super()._schedule_broadcasts(state, orbitdata)

    @runtime_tracker
    def __schedule_periodic_replan(self, state : SimulationAgentState, observations : list, maneuvers : list) -> list:
        """ Creates and schedules a waitForMessage action such that it triggers a periodic replan """
        
        # calculate next period for planning
        t_next = state.t + self.period

        # find wait start time
        if not observations and not maneuvers:
            t_wait_start = state.t 
        
        else:
            prelim_actions = [action for action in maneuvers]
            prelim_actions.extend(observations)

            actions_in_period = [action for action in prelim_actions 
                                 if  isinstance(action, AgentAction)
                                 and action.t_start < t_next]

            if actions_in_period:
                last_action : AgentAction = actions_in_period.pop()
                t_wait_start = min(last_action.t_end, t_next)
                                
            else:
                t_wait_start = state.t

        # create wait action
        return [WaitForMessages(t_wait_start, t_next)]

class AbstractReplanner(AbstractPlanner):
    """ Repairs plans previously constructed by another planner """

    @abstractmethod
    def update_percepts(self, 
                        state : SimulationAgentState,
                        current_plan : Plan,
                        incoming_reqs: list, 
                        relay_messages: list, 
                        misc_messages : list,
                        completed_actions: list,
                        aborted_actions : list,
                        pending_actions : list
                        ) -> None:
        
        super().update_percepts(state, incoming_reqs, relay_messages, completed_actions)
        
        # update latest preplan
        if abs(state.t - current_plan.t) <= 1e-3 and isinstance(current_plan, Preplan): 
            self.preplan = current_plan.copy() 

    @abstractmethod
    def generate_plan(  self, 
                        state : SimulationAgentState,
                        specs : object,
                        current_plan : Plan,
                        clock_config : ClockConfig,
                        orbitdata : OrbitData,
                    ) -> Plan:
        pass
        