
import math
import queue
from typing import Callable, Any

from nodes.planning.plan import Plan, Preplan
from nodes.orbitdata import OrbitData, TimeInterval
from nodes.states import *
from nodes.science.reqs import *
from messages import *
from dmas.modules import *
from dmas.utils import runtime_tracker
import pandas as pd


class AbstractPlanner(ABC):
    def __init__(self, 
                 utility_func : Callable[[], Any], 
                 logger : logging.Logger = None
                 ) -> None:
        
        # initialize object
        super().__init__()

        # initialize valiables
        self.generated_reqs = []
        self.completed_requests = []
        self.completed_broadcasts = []
        self.completed_actions = []
        self.pending_relays = []
        self.access_times = {}
        self.known_reqs = []
        self.stats = {}
        self.plan : Plan = None

        # set parameters
        self.utility_func = utility_func    # utility function
        self._logger = logger               # logger for debugging

    def update_precepts(self, 
                        state : SimulationAgentState,
                        current_plan : Plan,
                        completed_actions : list,
                        aborted_actions : list,
                        pending_actions : list,
                        incoming_reqs : list,
                        generated_reqs : list,
                        relay_messages : list,
                        misc_messages : list,
                        orbitdata : dict = None
                        ) -> None:
        
        """ Uses incoming precepts to update internal knowledge of the environment and the state of the parent agent """
        # update list of requests generated by the parent agent 
        self.generated_reqs.extend([req for req in generated_reqs 
                                    if isinstance(req, MeasurementRequest)
                                    and req not in self.generated_reqs
                                    and req not in self.known_reqs
                                    and req.s_max > 0.0])

        # update list of known requests
        new_reqs : list = self.__get_new_requests(incoming_reqs, generated_reqs)
        self.known_reqs.extend(new_reqs)

        # update list of performed broadcasts
        self.completed_broadcasts.extend([message_from_dict(**action.msg) 
                                          for action in completed_actions 
                                          if isinstance(action, BroadcastMessageAction)
                                          and message_from_dict(**action.msg) not in self.completed_broadcasts])

        # update list of relays to perform
        pending_relay_ids = [msg.id for msg in self.pending_relays]
        self.pending_relays.extend([relay_message 
                                    for relay_message in relay_messages 
                                    if relay_message.id not in pending_relay_ids])

        for completed_broadcast in self.completed_broadcasts:
            completed_broadcast : SimulationMessage
            for pending_relay in self.pending_relays:
                pending_relay : SimulationMessage
                if completed_broadcast.id == pending_relay.id:
                    self.pending_relays.remove(pending_relay)
                    break

        # update list of performed actions
        self.completed_actions.extend([action for action in completed_actions])

        # update list of performed requests
        completed_requests : list = self.__update_performed_requests(completed_actions, misc_messages)
        self.completed_requests.extend([req for req in completed_requests 
                                        if req not in self.completed_requests])
        
        # update access times 
        self._update_access_times(state, orbitdata[state.agent_name])
        
    @runtime_tracker
    def __get_new_requests( self, 
                            incoming_reqs : list,
                            generated_reqs : list
                            ) -> list:
        """
        Reads incoming requests and determines which ones are new or known
        """
        reqs = [req for req in incoming_reqs]
        reqs.extend(generated_reqs)
        return [req for req in reqs if req not in self.known_reqs and req.s_max > 0]

    @runtime_tracker
    def __update_performed_requests(self, performed_actions : list, misc_messages : list) -> list:
        """ Creates a list the of requests that were just performed by the parent agent or by other agents """
        performed_requests = []

        # compile measurements performed by parent agent
        performed_measurements = [action for action in performed_actions 
                                  if isinstance(action, MeasurementAction)]
        
        # compile measurements performed by other agents
        their_measurements = [MeasurementAction(**msg.measurement_action)
                              for msg in misc_messages 
                              if isinstance(msg, MeasurementPerformedMessage)]
                
        # compile performed measurements  
        performed_measurements.extend(their_measurements)

        # check if measurements are attributed to a known measurement request
        for action in performed_measurements:
            action : MeasurementAction 
            req : MeasurementRequest = MeasurementRequest.from_dict(action.measurement_req)
            if (action.status == action.COMPLETED 
                ):
                performed_requests.append((req, action.instrument_name))

        return performed_requests

    @abstractmethod
    def _update_access_times(  self,
                                state : SimulationAgentState,
                                t_plan : float,
                                agent_orbitdata : OrbitData) -> None:
        """
        Calculates and saves the access times of all known requests
        """
        pass

    @runtime_tracker
    def _calc_arrival_times(self, 
                            state : SimulationAgentState, 
                            req : MeasurementRequest, 
                            instrument : str,
                            t_prev : Union[int, float],
                            agent_orbitdata : OrbitData) -> float:
        """
        Estimates the quickest arrival time from a starting position to a given final position
        """
        if isinstance(req, GroundPointMeasurementRequest):
            # compute earliest time to the task
            if isinstance(state, SatelliteAgentState):
                t_imgs = []
                lat,lon,_ = req.lat_lon_pos
                t_start = min( max(t_prev, req.t_start), t_prev + self.horizon)     # TODO generalize
                t_end = min(t_prev + self.horizon, req.t_end)
                df : pd.DataFrame = agent_orbitdata \
                                        .get_ground_point_accesses_future(lat, lon, instrument, t_start, t_end)

                for _, row in df.iterrows():
                    t_img = row['time index'] * agent_orbitdata.time_step
                    dt = t_img - state.t
                
                    # propagate state
                    propagated_state : SatelliteAgentState = state.propagate(t_prev)

                    # compute off-nadir angle
                    thf = propagated_state.calc_off_nadir_agle(req)
                    dth = abs(thf - propagated_state.attitude[0])

                    # estimate arrival time using fixed angular rate TODO change to 
                    if dt >= dth / state.max_slew_rate: # TODO change maximum angular rate 
                        t_imgs.append(t_img)
                        
                return t_imgs

            elif isinstance(state, UAVAgentState):
                dr = np.array(req.pos) - np.array(state.pos)
                norm = np.sqrt( dr.dot(dr) )
                return [norm / state.max_speed + t_prev]

            else:
                raise NotImplementedError(f"arrival time estimation for agents of type `{type(state)}` is not yet supported.")

        else:
            raise NotImplementedError(f"cannot calculate imaging time for measurement requests of type {type(req)}")       

    @abstractmethod
    def needs_planning(self, **kwargs) -> bool:
        """ Determines whether planning is triggered """ 
        
    @abstractmethod
    def generate_plan(self, **kwargs) -> Plan:
        """ Creates a plan for the agent to perform """

    @runtime_tracker
    def _schedule_broadcasts(self, 
                             state : SimulationAgentState, 
                             measurements : list, 
                             orbitdata : dict
                            ) -> list:
        """ 
        Schedules any broadcasts to be done. 
        
        By default it schedules the broadcast of any newly generated requests
        and the relay of any incoming relay messages
        """
        # initialize list of broadcasts to be done
        broadcasts = []       

        # schedule generated measurement request broadcasts
        ## check which requests have not been broadcasted yet
        requests_broadcasted = [msg.req['id'] for msg in self.completed_broadcasts 
                                if isinstance(msg, MeasurementRequestMessage)]
        requests_to_broadcast = [req for req in self.generated_reqs
                                 if isinstance(req, MeasurementRequest)
                                 and req.id not in requests_broadcasted]

        # Find best path for broadcasts
        path, t_start = self._create_broadcast_path(state.agent_name, orbitdata, state.t)

        ## create a broadcast action for all unbroadcasted measurement requests
        for req in requests_to_broadcast:        
            # if found, create broadcast action
            msg = MeasurementRequestMessage(state.agent_name, state.agent_name, req.to_dict(), path=path)
            
            if t_start >= 0:
                broadcast_action = BroadcastMessageAction(msg.to_dict(), t_start)
                
                broadcasts.append(broadcast_action)

        # schedule message relay
        for relay in self.pending_relays:
            raise NotImplementedError('Relay scheduling not yet supported.')

            relay : SimulationMessage

            assert relay.path

            # find next destination and access time
            next_dst = relay.path[0]
            
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
                        
        # return scheduled broadcasts
        return broadcasts   

    @runtime_tracker
    def _schedule_maneuvers(    self, 
                                state : SimulationAgentState, 
                                observations : list,
                                broadcasts : list,
                                clock_config : ClockConfig
                            ) -> list:
        """
        Generates a list of AgentActions from the current path.

        Agents look to move to their designated measurement target and perform the measurement.

        ## Arguments:
            - state (:obj:`SimulationAgentState`): state of the agent at the start of the path
            - path (`list`): list of tuples indicating the sequence of observations to be performed and time of observation
            - t_init (`float`): start time for plan
            - clock_config (:obj:`ClockConfig`): clock being used for this simulation
        """

        # initialize maneuver list
        maneuvers = []

        for i in range(len(observations)):
            action_sequence_i = []

            measurement_action : MeasurementAction = observations[i]
            measurement_req = MeasurementRequest.from_dict(measurement_action.measurement_req)
            t_img = measurement_action.t_start

            if isinstance(state, SatelliteAgentState):
                imaging_state : SimulationAgentState = state.propagate(t_img)
            else:
                raise NotImplemented(f"maneuver scheduling for states of type `{type(state)}` not yet supported")

            if not isinstance(measurement_req, GroundPointMeasurementRequest):
                raise NotImplementedError(f"Cannot create plan for requests of type {type(measurement_req)}")
            
            # Estimate previous state
            if i == 0:
                if isinstance(state, SatelliteAgentState):
                    t_prev = state.t
                    prev_state : SatelliteAgentState = state.copy()

                # elif isinstance(state, UAVAgentState):
                #     t_prev = state.t # TODO consider wait time for convergence
                #     prev_state : UAVAgentState = state.copy()

                else:
                    raise NotImplemented(f"maneuver scheduling for states of type `{type(state)}` not yet supported")
            else:
                prev_measurement : MeasurementAction = observations[i-1]
                prev_req = MeasurementRequest.from_dict(prev_measurement.measurement_req)
                t_prev = prev_measurement.t_end if prev_measurement is not None else state.t

                if isinstance(state, SatelliteAgentState):
                    prev_state : SatelliteAgentState = state.propagate(t_prev)
                    prev_state.attitude = [
                                        prev_state.calc_off_nadir_agle(prev_req),
                                        0.0,
                                        0.0
                                    ]

                elif isinstance(state, UAVAgentState):
                    prev_state : UAVAgentState = state.copy()
                    prev_state.t = t_prev

                    if isinstance(prev_req, GroundPointMeasurementRequest):
                        prev_state.pos = prev_req.pos
                    else:
                        raise NotImplementedError(f"cannot calculate travel time start for requests of type {type(prev_req)} for uav agents")

                else:
                    raise NotImplementedError(f"cannot calculate travel time start for agent states of type {type(state)}")
                
            # maneuver to point to target
            t_maneuver_end = None
            if isinstance(state, SatelliteAgentState):
                prev_state : SatelliteAgentState
                imaging_state : SatelliteAgentState

                t_maneuver_start = prev_state.t
                th_f = imaging_state.calc_off_nadir_agle(measurement_req)
                dt = abs(th_f - prev_state.attitude[0]) / prev_state.max_slew_rate
                t_maneuver_end = t_maneuver_start + dt

                if t_maneuver_end > t_img:
                    x = 1

                if abs(t_maneuver_start - t_maneuver_end) >= 1e-3:
                    action_sequence_i.append(ManeuverAction([th_f, 0, 0], 
                                                            t_maneuver_start, 
                                                            t_maneuver_end))   
                else:
                    t_maneuver_end = None

            # move to target
            t_move_start = t_prev if t_maneuver_end is None else t_maneuver_end
            if isinstance(state, SatelliteAgentState):
                t_move_end = t_img
                future_state : SatelliteAgentState = state.propagate(t_move_end)
                final_pos = future_state.pos

            elif isinstance(state, UAVAgentState):
                final_pos = measurement_req.pos
                dr = np.array(final_pos) - np.array(prev_state.pos)
                norm = np.sqrt( dr.dot(dr) )
                
                t_move_end = t_move_start + norm / state.max_speed

            else:
                raise NotImplementedError(f"cannot calculate travel time end for agent states of type {type(state)}")
            
            # quantize travel maneuver times if needed
            if isinstance(clock_config, FixedTimesStepClockConfig):
                dt = clock_config.dt
                if t_move_start < np.Inf:
                    t_move_start = dt * math.floor(t_move_start/dt)
                if t_move_end < np.Inf:
                    t_move_end = dt * math.ceil(t_move_end/dt)
            
            # add travel maneuver if required
            if abs(t_move_start - t_move_end) >= 1e-3:
                move_action = TravelAction(final_pos, t_move_start, t_move_end)
                action_sequence_i.append(move_action)
            
            # wait for measurement action to start
            if t_move_end < t_img:
                action_sequence_i.append( WaitForMessages(t_move_end, t_img) )

            maneuvers.extend(action_sequence_i)

        return maneuvers
    
    def _create_broadcast_path(self, agent_name : str, orbitdata : dict, t : float) -> tuple:
        """ 
        Finds the best path for broadcasting a message to all agents using depth-first-search
        """
        # get list of agents
        agents = [agent for agent in orbitdata if agent != agent_name]
        
        # check if broadcast needs to be routed
        if agents:
            earliest_accesses = [orbitdata[agent].get_next_agent_access(agent_name, t) for agent in agents]           
            same_access_start = [access.start == earliest_accesses[0].start for access in earliest_accesses if isinstance(access, TimeInterval)]
            same_access_end = [access.end == earliest_accesses[0].end for access in earliest_accesses if isinstance(access, TimeInterval)]

            if all(same_access_start) and all(same_access_end):
                # all agents are accessing eachother at the same time; no need for mesasge relays
                return ([], t)   
            
        else:
            # no other agents in the simulation; no need for relays
            return ([], t)

        # initialte queue
        q = queue.Queue()
        
        # initialize min path and min path cost
        min_path = []
        min_times = []
        min_cost = np.Inf

        # add parent agent as the root node
        q.put((agent_name, [], [], 0.0))

        # iterate through depth-first search
        while not q.empty():
            # get next node in the search
            sender_agent, current_path, current_times, path_cost = q.get()

            # check if path is complete
            if len(agents) == len(current_path):
                # check if minimum cost
                if path_cost < min_cost:
                    min_cost = path_cost
                    min_path = [path_element for path_element in current_path]
                    min_times = [path_time for path_time in current_times]

            # add children nodes to queue
            for receiver_agent in [receiver_agent for receiver_agent in agents 
                                    if receiver_agent not in current_path
                                    and receiver_agent != agent_name
                                    ]:
                # query next access interval to children nodes
                t_access : float = t + path_cost

                sender_orbitdata : OrbitData = orbitdata[sender_agent]
                access_interval : TimeInterval = sender_orbitdata.get_next_agent_access(receiver_agent, t_access)
                
                if access_interval.start < np.Inf:
                    new_path = [path_element for path_element in current_path]
                    new_path.append(receiver_agent)

                    new_cost = access_interval.start - t

                    new_times = [path_time for path_time in current_times]
                    new_times.append(new_cost + t)

                    q.put((receiver_agent, new_path, new_times, new_cost))

        if min_times:
            assert t <= min_times[0]

        # return path and broadcast start time
        return (min_path, min_times[0]) if min_path else ([], np.Inf)
    
    def _print_observation_path(self, state : SatelliteAgentState, path : list) -> None :
        """ Debugging tool. Prints current observations plan being considered. """

        out = f'\n{state.agent_name}:\n\n\nID\t  j\tt_img\tth\tdt_mmt\tdt_mvr\tValid\tu_exp\n'

        out_temp = [f"N\A       ",
                    f"N\A",
                    f"\t{np.round(state.t,3)}",
                    f"\t{np.round(state.attitude[0],3)}",
                    f"\t-",
                    f"\t-",
                    f"\t-"
                    f"\t{0.0}",
                    f"\n"
                    ]
        out += ''.join(out_temp)

        for i in range(len(path)):
            if i > 0:
                measurement_prev : MeasurementAction = path[i-1]
                t_prev = measurement_prev.t_end
                req_prev : MeasurementRequest = MeasurementRequest.from_dict(measurement_prev.measurement_req)
                state_prev : SatelliteAgentState = state.propagate(t_prev)
                th_prev = state_prev.calc_off_nadir_agle(req_prev)
            else:
                t_prev = state.t
                state_prev : SatelliteAgentState = state
                th_prev = state.attitude[0]

            measurement_i : MeasurementAction = path[i]
            t_i = measurement_i.t_start
            req_i : MeasurementRequest = MeasurementRequest.from_dict(measurement_i.measurement_req)
            state_i : SatelliteAgentState = state.propagate(measurement_i.t_start)
            th_i = state_i.calc_off_nadir_agle(req_i)

            dt_maneuver = abs(th_i - th_prev) / state.max_slew_rate
            dt_measurements = t_i - t_prev

            out_temp = [f"{req_i.id.split('-')[0]}",
                            f"  {measurement_i.subtask_index}",
                            f"\t{np.round(measurement_i.t_start,3)}",
                            f"\t{np.round(th_i,3)}",
                            f"\t{np.round(dt_measurements,3)}",
                            f"\t{np.round(dt_maneuver,3)}",
                            f"\t{dt_maneuver <= dt_measurements}"
                            f"\t{np.round(measurement_i.u_exp,3)}",
                            f"\n"
                            ]
            out += ''.join(out_temp)
        out += f'\nn measurements: {len(path)}\n'

        print(out)