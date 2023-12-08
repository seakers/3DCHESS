from abc import ABC, abstractmethod
import logging
import math
from typing import Any, Callable
import pandas as pd

from dmas.utils import runtime_tracker
from dmas.clocks import *

from messages import *

from nodes.planning.plan import Plan
from nodes.orbitdata import OrbitData, TimeInterval
from nodes.states import *
from nodes.actions import *
from nodes.science.reqs import *
from nodes.science.utility import synergy_factor
from nodes.states import SimulationAgentState
from nodes.orbitdata import OrbitData

class AbstractPreplanner(ABC):
    """
    # Preplanner

    Conducts observations planning for an agent at the beginning of a planning horizon. 
    """
    def __init__(   self, 
                    utility_func : Callable[[], Any], 
                    horizon : float = np.Inf,
                    period : float = np.Inf,
                    logger: logging.Logger = None
                ) -> None:
        """
        ## Preplanner 
        
        Creates an instance of a preplanner class object.

        #### Arguments:
            - utility_func (`Callable`): desired utility function for evaluating observations
            - horizon (`float`) : planning horizon in seconds [s]
            - period (`float`) : period of replanning in seconds [s]
            - logger (`logging.Logger`) : debugging logger
        """
        super().__init__()

        # initialize valiables
        self.generated_reqs = []
        self.completed_requests = []
        self.completed_broadcasts = []
        self.access_times = {}
        self.known_reqs = []
        self.stats = {}
        self.plan = Plan()        

        # set properties
        self.utility_func = utility_func
        self.horizon = horizon
        self.period = period
        self._logger = logger

        # set 
        self.t_next = period

    @runtime_tracker
    def needs_initialized_plan( self, 
                                state : SimulationAgentState,
                                current_plan : Plan,
                                completed_actions : list,
                                aborted_actions : list,
                                pending_actions : list,
                                incoming_reqs : list,
                                generated_reqs : list,
                                misc_messages : list,
                                orbitdata : dict = None
                            ) -> bool:
        """ Determines whether a new plan needs to be initalized """    
        # update list of known requests
        new_reqs : list = self.__get_new_requests(incoming_reqs, generated_reqs)
        self.known_reqs.extend(new_reqs)

        # update list of requests generated by the parent agent
        self.generated_reqs.extend([req for req in generated_reqs 
                                        if isinstance(req, MeasurementRequest)
                                        and req not in self.generated_reqs])

        # update list of performed broadcasts
        self.completed_broadcasts.extend([action for action in completed_actions 
                                          if isinstance(action, BroadcastMessageAction)
                                          and action not in self.completed_broadcasts])

        # update list of performed requests
        completed_requests : list = self.__update_performed_requests(completed_actions, misc_messages)
        self.completed_requests.extend([req for req in completed_requests 
                                        if req not in self.completed_requests])
        
        # update access times 
        self.__update_access_times(state, current_plan.t_update, orbitdata[state.agent_name])
        
        # check if plan needs to be inialized
        return (current_plan.t_update < 0     # simulation just started
                or state.t >= self.t_next)    # planning horizon has been reached

    @runtime_tracker
    def __get_new_requests( self, 
                            incoming_reqs : list,
                            generated_reqs : list
                            ) -> list:
        """
        Reads incoming requests and determines which ones are new or known
        """
        reqs = [req for req in incoming_reqs]; reqs.extend(generated_reqs)
        return [req for req in reqs if req not in self.known_reqs and req.s_max > 0]

    @runtime_tracker
    def __update_performed_requests(self, performed_actions : list, misc_messages : list) -> list:
        """ Creates a list the of requests that were just performed by the parent agent or by other agents """
        performed_requests = []

        # compile measurements performed by parent agent
        performed_measurements = [action for action in performed_actions if isinstance(action, MeasurementAction)]
        
        # compile measurements performed by other agents
        their_measurements = [
                                MeasurementAction(**msg.measurement_action)
                                for msg in misc_messages if isinstance(msg,  MeasurementPerformedMessage)
                               ]
                
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

    @runtime_tracker
    def __update_access_times(  self,
                                state : SimulationAgentState,
                                t_plan : float,
                                agent_orbitdata : OrbitData) -> None:
        """
        Calculates and saves the access times of all known requests
        """
        if state.t >= self.t_next or t_plan < 0:
            # recalculate access times for all known requests            
            for req in self.known_reqs:
                req : MeasurementRequest
                self.access_times[req.id] = {instrument : [] for instrument in req.measurements}

                # check access for each required measurement
                for instrument in self.access_times[req.id]:
                    if instrument not in state.payload:
                        # agent cannot perform this request TODO add KG support
                        continue

                    if (req, instrument) in self.completed_requests:
                        # agent has already performed this request
                        continue

                    t_arrivals : list = self._calc_arrival_times(   state, 
                                                                    req, 
                                                                    instrument,
                                                                    state.t,
                                                                    agent_orbitdata)
                    self.access_times[req.id][instrument] = t_arrivals

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
                t_start = min( max(t_prev, req.t_start), t_prev + self.horizon)
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

    @runtime_tracker
    def initialize_plan(self, 
                        state : SimulationAgentState,
                        current_plan : list,
                        completed_actions : list,
                        aborted_actions : list,
                        pending_actions : list,
                        incoming_reqs : list,
                        generated_reqs : list,
                        misc_messages : list,
                        clock_config : ClockConfig,
                        orbitdata : dict = None
                    ) -> Plan:
        
        # schedule measurements
        measurements : list = self._schedule_measurements(state, clock_config)

        # generate maneuver and travel actions from measurements
        maneuvers : list = self._schedule_maneuvers(state, measurements, clock_config)

        # schedule broadcasts to be perfomed
        broadcasts : list = self._schedule_broadcasts(state, measurements, generated_reqs, orbitdata)
        
        # wait for next planning period to start
        replan : list = self.__schedule_periodic_replan(state, maneuvers, broadcasts)

        # generate plan from actions
        self.plan = Plan(measurements, maneuvers, broadcasts, replan, t=state.t)

        # update planning period time
        self.t_plan = state.t
        self.t_next = self.t_plan + self.period       

        # return plan
        return self.plan
        
    @abstractmethod
    def _schedule_measurements(self, state : SimulationAgentState, clock_config : ClockConfig) -> list:
        """ Creates a list of measurement actions to be performed by the agent """

    @runtime_tracker
    def _schedule_maneuvers(    self, 
                                state : SimulationAgentState, 
                                observations : list,
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

                t_maneuver_start = prev_state.t
                th_f = prev_state.calc_off_nadir_agle(measurement_req)
                dt = abs(th_f - prev_state.attitude[0]) / prev_state.max_slew_rate
                t_maneuver_end = t_maneuver_start + dt

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
            
            # add move maneuver if required
            if abs(t_move_start - t_move_end) >= 1e-3:
                move_action = TravelAction(final_pos, t_move_start, t_move_end)
                action_sequence_i.append(move_action)
            
            # wait for measurement action to start
            if t_move_end < t_img:
                action_sequence_i.append( WaitForMessages(t_move_end, t_img) )

            maneuvers.extend(action_sequence_i)

        return maneuvers

    def _schedule_broadcasts(self, 
                             state : SimulationAgentState, 
                             measurements : list, 
                             generated_reqs : list, 
                             orbitdata : dict
                            ) -> list:
        """ Schedules any broadcasts to be done. By default it only schedules pending messages from previous planning horizons """

        # generate a set of broadcasts to be performed 
        # initialize list of broadcasts to be done
        broadcasts = []

        # schedule generated measurement request broadcasts
        for req in self.generated_reqs:
            req : MeasurementRequest
            # check if this request has been broadcasted yet
            msg : MeasurementRequestMessage

            matching_broadcasts = [msg for msg in self.completed_broadcasts 
                                    if ]
            for msg in self.completed_broadcasts:
                x = 1

            x = 1


            pass
            # msg = MeasurementRequestMessage(state.agent_name, state.agent_name, req.to_dict())
            
            # # place broadcasts in plan
            # if isinstance(state, SatelliteAgentState):
            #     raise ValueError('TODO')
                
            #     if not orbitdata:
            #         raise ValueError('orbitdata required for satellite agents')
                
            #     # get next access windows to all agents
            #     isl_accesses = [orbitdata.get_next_agent_access(target, state.t) for target in orbitdata.isl_data]

            #     # TODO merge accesses to find overlaps
            #     isl_accesses_merged = isl_accesses

            #     # place requests in pending broadcasts list
            #     for interval in isl_accesses_merged:
            #         interval : TimeInterval
            #         broadcast_action = BroadcastMessageAction(msg.to_dict(), max(interval.start, state.t))

            #         broadcasts.append(broadcast_action)
                    
            # else:
            #     raise NotImplementedError(f"Scheduling of broadcasts for agents with state of type {type(state)} not yet implemented.")
        
        
        # schedule message relay

        # # include broadcasts to list of pending broadcasts if they are to be done after the next replanning horizon
        # self.pending_broadcasts.extend([broadcast_action for broadcast_action in broadcasts 
        #                                 if broadcast_action not in self.pending_broadcasts
        #                                 and broadcast_action.t_start > state.t + self.period])
        
        # # schedule pending broadcasts that can be performed within the next planning period
        # scheduled_broadcasts = [broadcast_action for broadcast_action in broadcasts 
        #                         if broadcast_action not in self.pending_broadcasts]
                
        # return scheduled broadcasts
        return broadcasts   

    def _generate_broadcasts(self, 
                             state : SimulationAgentState, 
                             measurements : list, 
                             generated_reqs : list, 
                             orbitdata : OrbitData
                             ) -> list:
        """ Creates broadcast actions """
        # initialize list of broadcasts to be done
        broadcasts = []

        # schedule generated measurement request broadcasts
        for req in generated_reqs:
            req : MeasurementRequest
            msg = MeasurementRequestMessage(state.agent_name, state.agent_name, req.to_dict())
            
            # place broadcasts in plan
            if isinstance(state, SatelliteAgentState):
                if not orbitdata:
                    raise ValueError('orbitdata required for satellite agents')
                
                # get next access windows to all agents
                isl_accesses = [orbitdata.get_next_agent_access(target, state.t) for target in orbitdata.isl_data]

                # TODO merge accesses to find overlaps
                isl_accesses_merged = isl_accesses

                # place requests in pending broadcasts list
                for interval in isl_accesses_merged:
                    interval : TimeInterval
                    broadcast_action = BroadcastMessageAction(msg.to_dict(), max(interval.start, state.t))

                    broadcasts.append(broadcast_action)
                    
            else:
                raise NotImplementedError(f"Scheduling of broadcasts for agents with state of type {type(state)} not yet implemented.")
        
        return broadcasts   

    def __schedule_periodic_replan(self, state : SimulationAgentState, measurement_actions : list, broadcast_actions : list) -> list:
        """ Creates and schedules a waitForMessage action such that it triggers a periodic replan """

        # raise NotImplementedError("TODO")

        # find wait start time
        if not measurement_actions and not broadcast_actions:
            t_wait_start = state.t 
        else:
            actions_in_period = [action for action in measurement_actions if action.t_start < state.t + self.period]

            if actions_in_period:
                last_action : AgentAction = actions_in_period.pop()
                if last_action.t_end < self.t_next:
                    t_wait_start = last_action.t_end
                else:
                    t_wait_start = state.t + self.period
                                
            else:
                t_wait_start = state.t

        # create wait action
        return [WaitForMessages(t_wait_start, state.t + self.period)]
        
    @runtime_tracker
    def _get_available_requests(self) -> list:
        """ Returns a list of known requests that can be performed within the current planning horizon """

        reqs = {req.id : req for req in self.known_reqs}
        available_reqs = []

        for req_id in self.access_times:
            req : MeasurementRequest = reqs[req_id]
            for instrument in self.access_times[req_id]:
                t_arrivals : list = self.access_times[req_id][instrument]

                if len(t_arrivals) > 0:
                    for subtask_index in range(len(req.measurement_groups)):
                        main_instrument, _ = req.measurement_groups[subtask_index]
                        if main_instrument == instrument:
                            available_reqs.append((reqs[req_id], subtask_index))
                            break

        return available_reqs

class IdlePlanner(AbstractPreplanner):
    @runtime_tracker
    def initialize_plan(    self, 
                            *_
                        ) -> tuple:
        return [IdleAction(0.0, np.Inf)]

class FIFOPreplanner(AbstractPreplanner):
    def __init__(self, 
                 utility_func: Callable[[], Any], 
                 period : float = np.Inf,
                 horizon : float = np.Inf,
                 collaboration : bool = False,
                 logger: logging.Logger = None, 
                 **kwargs
                 ) -> None:
        """
        ### First Come, First Served Preplanner

        Schedules 
        """

        super().__init__(utility_func, horizon, period, logger)
        self.collaboration = collaboration    

    def _schedule_measurements(self, state : SimulationAgentState, clock_config : ClockConfig) -> list:
        """ 
        Schedule a sequence of observations based on the current state of the agent 
        
        ### Arguments:
            - state (:obj:`SimulationAgentState`): state of the agent at the time of planning
        """
        
        # initialize measurement path
        measurements = []         

        # get available requests
        available_reqs : list = self._get_available_requests()

        if isinstance(state, SatelliteAgentState):

            # create first assignment of observations
            for req, subtask_index in available_reqs:
                req : MeasurementRequest
                main_measurement, _ = req.measurement_groups[subtask_index]  
                t_arrivals : list = self.access_times[req.id][main_measurement]

                if len(t_arrivals) > 0:
                    t_img = t_arrivals.pop(0)
                    u_exp = self.utility_func(req.to_dict(), t_img) * synergy_factor(req.to_dict(), subtask_index)

                    # perform measurement
                    measurements.append(MeasurementAction( req.to_dict(),
                                                   subtask_index, 
                                                   main_measurement,
                                                   u_exp,
                                                   t_img, 
                                                   t_img + req.duration
                                                 ))

            # sort from ascending start time
            measurements.sort(key=lambda a: a.t_start)

            # ----- FOR DEBUGGING PURPOSES ONLY ------
            # self.__print_observation_path(state, measurements)
            # ----------------------------------------

            # ensure conflict-free path
            while True:                
                conflict_free = True
                i_remove = None

                for i in range(len(measurements) - 1):
                    j = i + 1
                    measurement_i : MeasurementAction = measurements[i]
                    measurement_j : MeasurementAction = measurements[j]

                    req_i : MeasurementRequest = MeasurementRequest.from_dict(measurement_i.measurement_req)
                    req_j : MeasurementRequest = MeasurementRequest.from_dict(measurement_j.measurement_req)

                    th_i = state.calc_off_nadir_agle(req_i)
                    th_j = state.calc_off_nadir_agle(req_j)

                    dt_maneuver = abs(th_i - th_j) / state.max_slew_rate
                    dt_measurements = measurement_j.t_start - measurement_i.t_end

                    # check if there's enough time to maneuver from one observation to another
                    if dt_maneuver > dt_measurements:

                        # there is not enough time to maneuver 
                        main_measurement, _ = req_j.measurement_groups[measurement_j.subtask_index]  
                        t_arrivals : list = self.access_times[req_j.id][main_measurement]
                        
                        if len(t_arrivals) > 0:     # try again for the next access period
                            # pick next access time
                            t_img = t_arrivals.pop(0)
                            u_exp = self.utility_func(req.to_dict(), t_img) * synergy_factor(req.to_dict(), subtask_index)

                            # update measurement action
                            measurement_j.t_start = t_img
                            measurement_j.t_end = t_img + req_j.duration
                            measurement_j.u_exp = u_exp
                            
                            # update path list
                            measurements[j] = measurement_j

                            # sort from ascending start time
                            measurements.sort(key=lambda a: a.t_start)
                        else:                       # no more future accesses for this GP
                            i_remove = j
                            conflict_free = False
                            # break
                            raise Exception("Whoops. See Plan Initializer.")

                        # flag current observation plan as unfeasible for rescheduling
                        conflict_free = False
                        break
                
                if i_remove is not None:
                    measurements.pop(j) 

                if conflict_free:
                    break
                    
            # ----- FOR DEBUGGING PURPOSES ONLY ------
            # self.__print_observation_path(state, measurements)
            # ----------------------------------------
            
        else:
            raise NotImplementedError(f'initial planner for states of type `{type(state)}` not yet supported')
    
        return measurements

    def __print_observation_path(self, state : SatelliteAgentState, path : list) -> None :
        """ Debugging tool. Prints current observations plan being considered. """

        out = '\n\n\nID\t  Subtask\tt_img\tdt_mmt\tdt_mvr\tValid\tu_exp\n'
        for i in range(len(path) - 1):
            if i == 0:
                measurement_i : MeasurementAction = path[i]
                req_i : MeasurementRequest = MeasurementRequest.from_dict(measurement_i.measurement_req)

                out_temp = [f"{req_i.id.split('-')[0]}",
                            f"\t{measurement_i.subtask_index}",
                            f"\t{np.round(measurement_i.t_start,3)}",
                            f"\t-",
                            f"\t-",
                            f"\tTrue"
                            f"\t{np.round(measurement_i.u_exp,3)}",
                            f"\n"
                            ]
                out += ''.join(out_temp)
                continue 
            
            j = i - 1 
            measurement_i : MeasurementAction = path[i]
            measurement_prev : MeasurementAction = path[j]

            req_i : MeasurementRequest = MeasurementRequest.from_dict(measurement_i.measurement_req)
            req_prev : MeasurementRequest = MeasurementRequest.from_dict(measurement_prev.measurement_req)

            th_i = state.calc_off_nadir_agle(req_i)
            th_prev = state.calc_off_nadir_agle(req_prev)

            dt_maneuver = abs(th_i - th_prev) / state.max_slew_rate
            dt_measurements = measurement_i.t_start - measurement_prev.t_end

            out_temp = [f"{req_i.id.split('-')[0]}",
                            f"\t{measurement_i.subtask_index}",
                            f"\t{np.round(measurement_i.t_start,3)}",
                            f"\t{np.round(dt_measurements,3)}",
                            f"\t{np.round(dt_maneuver,3)}",
                            f"\t{dt_maneuver <= dt_measurements}"
                            f"\t{np.round(measurement_i.u_exp,3)}",
                            f"\n"
                            ]
            out += ''.join(out_temp)

        print(out)
    
    @runtime_tracker
    def _generate_broadcasts(self, 
                             state: SimulationAgentState,
                             measurements: list, 
                             generated_reqs: list, 
                             orbitdata: OrbitData
                            ) -> list:
        # initialize broadcast list
        broadcasts : list = super()._generate_broadcasts(state, measurements, generated_reqs, orbitdata)

        if self.collaboration:
            # schedule measurement action completion broadcasts
            planned_measurements = [action for action in measurements 
                                    if isinstance(action, MeasurementAction)]
            
            for action in planned_measurements:
                action : MeasurementAction

                # generate broadcast message
                action_copy = action_from_dict(**action.to_dict())
                action_copy.status = MeasurementAction.COMPLETED
                msg = MeasurementPerformedMessage(state.agent_name, state.agent_name, action_copy.to_dict())
                
                # place broadcasts in plan
                if isinstance(state, SatelliteAgentState):
                    if not orbitdata:
                        raise ValueError('orbitdata required for satellite agents')
                    
                    # get next access windows to all agents
                    isl_accesses = [orbitdata.get_next_agent_access(target, action.t_end) for target in orbitdata.isl_data]

                    # TODO merge accesses to find overlaps
                    isl_accesses_merged = isl_accesses

                    # handle placement in plan
                    for interval in isl_accesses_merged:
                        interval : TimeInterval
                        broadcast_action = BroadcastMessageAction(msg.to_dict(), max(interval.start, action.t_end))

                        if self.t_next < broadcast_action.t_start:
                            broadcasts.append(broadcast_action)
                            continue

                        # place broadcast action in plan
                        broadcasts.append(broadcast_action)
                        
                else:
                    raise NotImplementedError(f"Scheduling of broadcasts for agents with state of type {type(state)} not yet implemented.")

        # return broadcast list
        return broadcasts