import logging

from dmas.clocks import ClockConfig
from dmas.utils import runtime_tracker
from dmas.clocks import *

from chess3d.agents.orbitdata import OrbitData, TimeInterval
from chess3d.agents.states import *
from chess3d.agents.actions import *
from chess3d.agents.science.requests import *
from chess3d.agents.states import SimulationAgentState
from chess3d.agents.orbitdata import OrbitData
from chess3d.agents.planning.planner import AbstractPreplanner
from chess3d.messages import *

class NaivePlanner(AbstractPreplanner):
    """ Schedules observations based on the earliest feasible access point and broadcasts plan to all agents in the network """

    @runtime_tracker
    def _schedule_observations(self, 
                               state: SimulationAgentState, 
                               _: ClockConfig, 
                               orbitdata: OrbitData = None
                               ) -> list:
        if not isinstance(state, SatelliteAgentState):
            raise NotImplementedError(f'Naive planner not yet implemented for agents of type `{type(state)}.`')

        # compile access times for this planning horizon
        access_times, ground_points = self.calculate_access_times(state, orbitdata)
        access_times : list; ground_points : dict
        
        # generate plan
        observations = []
        while access_times:
            # get next available access interval
            grid_index, gp_index, instrument, _, t, th = access_times.pop()
            lat,lon = ground_points[grid_index][gp_index]
            target = [lat,lon,0.0]

            # check feasibility 
            if not observations:
                # no prior observation exists, compare with current state
                t_prev = state.t
                th_prev = state.attitude[0]
            else:
                # compare with previous scheduled observation
                action_prev : ObservationAction = observations[-1]
                t_prev = action_prev.t_end
                th_prev = action_prev.look_angle
          
            # find if feasible observation time exists 
            feasible_obs = [(t[i], th[i]) 
                            for i in range(len(t))
                            if self.is_observation_feasible(t[i], th[i], t_prev, th_prev)]
            
            if feasible_obs:
                # is feasible; create observation action
                t_img, th_img = feasible_obs[0]
                action = ObservationAction(instrument, target, th_img, t_img)
                observations.append(action)

        assert self.no_redundant_observations(state, observations, orbitdata)

        return observations
    
    def is_observation_feasible(self, t_img, th_img, t_prev, th_prev) -> bool:
        """ compares previous observation """
        # calculate inteval between observations
        dt_obs = t_img - t_prev

        # estimate maneuver time
        dt_maneuver = abs(th_img - th_prev)

        # check feasibility
        return dt_maneuver <= dt_obs
    
    def no_redundant_observations(self, 
                                 state : SimulationAgentState, 
                                 observations : list,
                                 orbitdata : OrbitData
                                 ) -> bool:
        if isinstance(state, SatelliteAgentState):
            for j in range(len(observations)):
                i = j - 1

                if i < 0: # there was no prior observation performed
                    continue                

                observation_prev : ObservationAction = observations[i]
                observation_curr : ObservationAction = observations[j]

                if (
                    abs(observation_curr.target[0] - observation_prev.target[0]) <= 1e-3
                    and abs(observation_curr.target[1] - observation_prev.target[1]) <= 1e-3
                    and (observation_curr.t_start - observation_prev.t_end) <= orbitdata.time_step):
                    return False
            
            return True

        else:
            raise NotImplementedError(f'Measurement path validity check for agents with state type {type(state)} not yet implemented.')
            
    @runtime_tracker
    def calculate_access_times(self, state : SimulationAgentState, orbitdata : OrbitData) -> dict:
        # define planning horizon
        t_start = state.t
        t_end = self.plan.t_next+self.horizon
        t_index_start = t_start / orbitdata.time_step
        t_index_end = t_end / orbitdata.time_step

        # compile coverage data
        raw_coverage_data = [(t_index*orbitdata.time_step, *_)
                             for t_index, *_ in orbitdata.gp_access_data.values
                             if t_index_start <= t_index <= t_index_end]
                
        # initiate accestimes 
        access_times = {}
        ground_points = {}
        
        for t_img, gp_index, _, lat, lon, _, look_angle, _, _, grid_index, instrument, _ in raw_coverage_data:
        # for t_index,_,_,lat_access,lon_access,_,th_img,*_ in accesses.values:
            
            # initialize dictionaries if needed
            if grid_index not in access_times:
                access_times[grid_index] = {}
                ground_points[grid_index] = {}
            if gp_index not in access_times[grid_index]:
                access_times[grid_index][gp_index] = {instrument : [] 
                                                        for instrument in state.payload}
                ground_points[grid_index][gp_index] = (lat, lon)

            # compile time interval information 
            found = False
            for interval, t, th in access_times[grid_index][gp_index][instrument]:
                interval : TimeInterval
                t : list
                th : list

                if (interval.is_during(t_img - orbitdata.time_step) 
                    or interval.is_during(t_img + orbitdata.time_step)):
                    interval.extend(t_img)
                    t.append(t_img)
                    th.append(look_angle)
                    found = True
                    break                        

            if not found:
                access_times[grid_index][gp_index][instrument].append([TimeInterval(t_img, t_img), [t_img], [look_angle]])

        # convert to `list`
        access_times = [    (grid_index, gp_index, instrument, interval, t, th)
                            for grid_index in access_times
                            for gp_index in access_times[grid_index]
                            for instrument in access_times[grid_index][gp_index]
                            for interval, t, th in access_times[grid_index][gp_index][instrument]
                        ]
        
        # sort by observation time
        access_times.sort(key = lambda a: a[3],reverse=True)
        
        # return access times and grid information
        return access_times, ground_points
    
    @runtime_tracker
    def _schedule_broadcasts(self, 
                             state: SimulationAgentState, 
                             observations : list, 
                             orbitdata: OrbitData) -> list:
        # schedule relays 
        broadcasts : list = super()._schedule_broadcasts(state, observations, orbitdata)

        # gather observation plan to be sent out
        plan_out = [action.to_dict()
                    for action in observations
                    if isinstance(action,ObservationAction)]

        # check if observations exist in plan
        if plan_out:
            # find best path for broadcasts
            path, t_start = self._create_broadcast_path(state, orbitdata)

            # check feasibility of path found
            if t_start >= 0:
                # create plan message
                msg = PlanMessage(state.agent_name, state.agent_name, plan_out, state.t, path=path)
                
                # add plan broadcast to list of broadcasts
                broadcasts.append(BroadcastMessageAction(msg.to_dict(),t_start))
                
                # add action performance broadcast to plan
                for action_dict in plan_out:
                    path, t_start = self._create_broadcast_path(state, orbitdata, action_dict['t_end'])
                    msg = ObservationPerformedMessage(state.agent_name, state.agent_name, action_dict)
                    if t_start >= 0: broadcasts.append(BroadcastMessageAction(msg.to_dict(),t_start))

        # return broadcast plan
        return broadcasts