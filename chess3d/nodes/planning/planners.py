import os
import re
import time
from typing import Any, Callable
from nodes.planning.preplanners import AbstractPreplanner
from nodes.planning.replanners import AbstractReplanner
from nodes.orbitdata import OrbitData
from nodes.states import *
from nodes.science.reqs import *
from messages import *
from dmas.modules import *
import pandas as pd

class PlanningModule(InternalModule):
    def __init__(self, 
                results_path : str, 
                parent_name : str,
                parent_network_config: NetworkConfig, 
                utility_func : Callable[[], Any],
                preplanner : AbstractPreplanner = None,
                replanner : AbstractReplanner = None,
                initial_reqs : list = [],
                level: int = logging.INFO, 
                logger: logging.Logger = None
                ) -> None:
                       
        # setup network settings
        addresses = parent_network_config.get_internal_addresses()        
        sub_addesses = []
        sub_address : str = addresses.get(zmq.PUB)[0]
        sub_addesses.append( sub_address.replace('*', 'localhost') )

        if len(addresses.get(zmq.SUB)) > 1:
            sub_address : str = addresses.get(zmq.SUB)[1]
            sub_addesses.append( sub_address.replace('*', 'localhost') )

        pub_address : str = addresses.get(zmq.SUB)[0]
        pub_address = pub_address.replace('localhost', '*')

        addresses = parent_network_config.get_manager_addresses()
        push_address : str = addresses.get(zmq.PUSH)[0]

        planner_network_config =  NetworkConfig(parent_name,
                                        manager_address_map = {
                                        zmq.REQ: [],
                                        zmq.SUB: sub_addesses,
                                        zmq.PUB: [pub_address],
                                        zmq.PUSH: [push_address]})

        # intialize module
        super().__init__(f'{parent_name}-PLANNING_MODULE', 
                        planner_network_config, 
                        parent_network_config, 
                        level, 
                        logger)
        
        # initialize default attributes
        self.results_path = results_path
        self.parent_name = parent_name
        self.utility_func = utility_func

        self.preplanner : AbstractPreplanner = preplanner
        self.replanner : AbstractReplanner = replanner
        
        self.plan_history = []
        self.plan = []
        self.stats = {
                        "planning" : [],
                        "preplanning" : [],
                        "replanning" : [],
                        "science_wait" : []
                    }

        self.agent_state : SimulationAgentState = None
        self.parent_agent_type = None
        self.orbitdata : OrbitData = None
        self.other_modules_exist : bool = False

        self.initial_reqs = []
        for req in initial_reqs:
            req : MeasurementRequest
            if req.t_start > 0:
                continue
            self.initial_reqs.append(req)

    async def sim_wait(self, delay: float) -> None:
        # does nothing
        return

    async def setup(self) -> None:
        # initialize internal messaging queues
        self.states_inbox = asyncio.Queue()
        self.action_status_inbox = asyncio.Queue()
        self.req_inbox = asyncio.Queue()
        self.measurement_inbox = asyncio.Queue()
        self.misc_inbox = asyncio.Queue()

        # setup agent state locks
        self.agent_state_lock = asyncio.Lock()
        self.agent_state_updated = asyncio.Event()

    async def live(self) -> None:
        """
        Performs two concurrent tasks:
        - Listener: receives messages from the parent agent and checks results
        - Bundle-builder: plans and bids according to local information
        """
        try:
            listener_task = asyncio.create_task(self.listener(), name='listener()')
            bundle_builder_task = asyncio.create_task(self.planner(), name='planner()')
            
            tasks = [listener_task, bundle_builder_task]

            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)

        finally:
            for task in done:
                self.log(f'`{task.get_name()}` task finalized! Terminating all other tasks...')

            for task in pending:
                task : asyncio.Task
                if not task.done():
                    task.cancel()
                    await task

    async def listener(self) -> None:
        """
        Listens for any incoming messages, unpacks them and classifies them into 
        internal inboxes for future processing
        """
        try:
            # listen for broadcasts and place in the appropriate inboxes
            while True:
                self.log('listening to manager broadcast!')
                _, _, content = await self.listen_manager_broadcast()

                # if sim-end message, end agent `live()`
                if content['msg_type'] == ManagerMessageTypes.SIM_END.value:
                    self.log(f"received manager broadcast or type {content['msg_type']}! terminating `live()`...")
                    return

                elif content['msg_type'] == SimulationMessageTypes.SENSES.value:
                    # received agent sense message from parent
                    self.log(f"received senses from parent agent!", level=logging.DEBUG)

                    # unpack message 
                    senses_msg : SensesMessage = SensesMessage(**content)

                    senses = []
                    senses.append(senses_msg.state)
                    senses.extend(senses_msg.senses)     

                    for sense in senses:
                        if sense['msg_type'] == SimulationMessageTypes.AGENT_ACTION.value:
                            # unpack message 
                            action_msg = AgentActionMessage(**sense)
                            self.log(f"received agent action of status {action_msg.status}!")
                            
                            # send to planner
                            await self.action_status_inbox.put(action_msg)

                        elif sense['msg_type'] == SimulationMessageTypes.AGENT_STATE.value:
                            # unpack message 
                            state_msg : AgentStateMessage = AgentStateMessage(**sense)
                            self.log(f"received agent state message!")
                                                        
                            # update current state
                            await self.agent_state_lock.acquire()
                            state : SimulationAgentState = SimulationAgentState.from_dict(state_msg.state)

                            await self.update_current_time(state.t)
                            self.agent_state = state

                            if self.parent_agent_type is None:
                                if isinstance(state, SatelliteAgentState):
                                    # import orbit data
                                    self.orbitdata : OrbitData = self._load_orbit_data()
                                    self.parent_agent_type = SimulationAgentTypes.SATELLITE.value
                                elif isinstance(state, UAVAgentState):
                                    self.parent_agent_type = SimulationAgentTypes.UAV.value
                                elif isinstance(state, GroundStationAgentState):
                                    self.parent_agent_type = SimulationAgentTypes.GROUND_STATION.value
                                else:
                                    raise NotImplementedError(f"states of type {state_msg.state['state_type']} not supported for planners.")
                            
                            self.agent_state_lock.release()
                            await self.states_inbox.put(state)

                        elif sense['msg_type'] == SimulationMessageTypes.MEASUREMENT_REQ.value:
                            # request received directly from another agent
                            
                            # unpack message 
                            req_msg = MeasurementRequestMessage(**sense)
                            req : MeasurementRequest = MeasurementRequest.from_dict(req_msg.req)
                            self.log(f"received measurement request message!")
                            
                            # send to planner
                            await self.req_inbox.put(req)

                        # TODO support down-linked information processing

                        elif sense['msg_type'] == SimulationMessageTypes.MEASUREMENT.value:
                            # measurement was just performed by agent
                            msg = message_from_dict(**sense)
                            await self.measurement_inbox.put(msg)

                        else:
                            # other type of message was received
                            msg = message_from_dict(**sense)
                            await self.misc_inbox.put(msg)

                # elif content['msg_type'] == SimulationMessageTypes.MEASUREMENT_REQ.value:
                #     # request received directly from another module

                #     # unpack message 
                #     req_msg = MeasurementRequestMessage(**content)
                #     req : MeasurementRequest = MeasurementRequest.from_dict(req_msg.req)
                #     self.log(f"received measurement request message from another module!")
                    
                #     # send to planner
                #     await self.req_inbox.put(req)

                else:
                    # other type of message was received
                    if not self.other_modules_exist:
                        self.other_modules_exist = True

                    msg = message_from_dict(**content)
                    await self.internal_inbox.put(msg)

        except asyncio.CancelledError:
            return

    async def planner(self) -> None:
        """
        Generates a plan for the parent agent to perform
        """
        try:
            # initialize results vectors
            plan = []

            # level = logging.WARNING
            level = logging.DEBUG

            while True:
                # wait for agent to update state
                _ : AgentStateMessage = await self.states_inbox.get()

                # --- Check Action Completion ---
                performed_actions = []
                while not self.action_status_inbox.empty():
                    action_msg : AgentActionMessage = await self.action_status_inbox.get()
                    action : AgentAction = action_from_dict(**action_msg.action)

                    if action_msg.status == AgentAction.PENDING:
                        # if action wasn't completed, try again
                        plan_ids = [action.id for action in self.plan]
                        action_dict : dict = action_msg.action
                        if action_dict['id'] in plan_ids:
                            self.log(f'action {action_dict} not completed yet! trying again...')
                            plan_out.append(action_dict)

                    else:
                        # if action was completed or aborted, remove from plan
                        performed_actions.append(action)

                        if action_msg.status == AgentAction.COMPLETED:
                            self.log(f'action of type `{action.action_type}` completed!', level)

                        action_dict : dict = action_msg.action
                        completed_action = AgentAction(**action_dict)
                        removed = None
                        for action in plan:
                            action : AgentAction
                            if action.id == completed_action.id:
                                removed = action
                                break
                        
                        if removed is not None:
                            removed : AgentAction
                            plan : list
                            plan.remove(removed)
                
                # --- Look for Plan Updates ---
                #                 
                # check if plan has been initialized
                if (len(self.initial_reqs) > 0                  # there are initial requests 
                    and self.get_current_time() <= 1e-3         # simulation just started
                    and self.preplanner is not None             # there is a preplanner assigned to this planner
                    ):
                    # Generate Initial Plan
                    initial_reqs = self.initial_reqs
                    self.initial_reqs = []
                    
                    await self.agent_state_lock.acquire()
                    t_0 = time.perf_counter()
                    plan : list = self.preplanner.initialize_plan(  self.agent_state, 
                                                                    initial_reqs,
                                                                    self.orbitdata,
                                                                    self._clock_config,
                                                                    level
                                                                    )
                    dt = time.perf_counter() - t_0
                    self.stats['preplanning'].append(dt)
                    self.agent_state_lock.release()

                # Check incoming messages
                t_0 = time.perf_counter()
                incoming_reqs = []
                while not self.req_inbox.empty():
                    incoming_reqs.append(await self.req_inbox.get())

                incoming_measurements = []
                while not self.measurement_inbox.empty():
                    incoming_measurements.append(await self.measurement_inbox.get())

                incoming_misc = []
                while not self.misc_inbox.empty():
                    incoming_misc.append(await self.misc_inbox.get())

                generated_reqs = []
                if len(incoming_measurements) > 0 and self.other_modules_exist:
                    generated_reqs.append( await self.internal_inbox.get())
                    while not self.misc_inbox.empty():
                        generated_reqs.append(await self.internal_inbox.get())
                dt = time.perf_counter() - t_0
                self.stats['science_wait'].append(dt)
                
                # Check if reeplanning is needed
                await self.agent_state_lock.acquire()
                t_0 = time.perf_counter()
                if (
                    self.replanner is not None and 
                    self.replanner.needs_replanning(self.agent_state,
                                                    plan, 
                                                    incoming_reqs,
                                                    generated_reqs,
                                                    incoming_misc)
                    ):
                    plan : list = self.replanner.revise_plan(   self.agent_state,
                                                                plan, 
                                                                incoming_reqs,
                                                                generated_reqs,
                                                                incoming_misc,
                                                                self.orbitdata,
                                                                level
                                                            )
                    
                    dt = time.perf_counter() - t_0
                    self.stats['replanning'].append(dt)

                self.agent_state_lock.release()

                # --- Execute plan ---

                # get next action to perform
                t_0 = time.perf_counter()
                plan_out = self.get_next_actions(plan, self.get_current_time())
                dt = time.perf_counter() - t_0
                self.stats['planning'].append(dt)

                # --- FOR DEBUGGING PURPOSES ONLY: ---
                out = f'\nPLAN\nid\taction type\tt_start\tt_end\n'
                for action in plan:
                    action : AgentAction
                    out += f"{action.id.split('-')[0]}, {action.action_type}, {action.t_start}, {action.t_end}\n"
                # self.log(out, level=logging.WARNING)

                out = f'\nPLAN OUT\nid\taction type\tt_start\tt_end\n'
                for action in plan_out:
                    action : dict
                    out += f"{action['id'].split('-')[0]}, {action['action_type']}, {action['t_start']}, {action['t_end']}\n"
                # self.log(out, level=logging.WARNING)
                # -------------------------------------

                self.log(f'sending {len(plan_out)} actions to agent...')
                plan_msg = PlanMessage(self.get_element_name(), self.get_network_name(), plan_out)
                await self._send_manager_msg(plan_msg, zmq.PUB)

                self.log(f'actions sent!')

        except asyncio.CancelledError:
            return
        
    def get_next_actions(self, plan : list, t : float) -> list:
        """ Parses current plan and outputs list of actions that are to be performed at a given time"""

        plan_out = []
        plan_out_ids = [action['id'] for action in plan_out]
        for action in plan:
            action : AgentAction
            if (action.t_start <= self.get_current_time()
                and action.id not in plan_out_ids):
                
                plan_out.append(action.to_dict())

                if len(plan_out_ids) > 0:
                    break

        if len(plan_out) == 0:
            if len(plan) > 0:
                # next action is yet to start, wait until then
                next_action : AgentAction = plan[0]
                t_idle = next_action.t_start if next_action.t_start > t else t
            else:
                # no more actions to perform
                if isinstance(self.agent_state, SatelliteAgentState):
                    # wait until the next ground-point access
                    self.orbitdata : OrbitData
                    # t_next = self.orbitdata.get_next_gs_access(t)
                    t_next = np.Inf
                else:
                    # idle until the end of the simulation
                    t_next = np.Inf

                # t_idle = self.t_plan + self.planning_horizon
                t_idle = t_next

            action = WaitForMessages(t, t_idle)
            plan_out.append(action.to_dict())

        return plan_out

    def _load_orbit_data(self) -> OrbitData:
        """
        Loads agent orbit data from pre-computed csv files in scenario directory
        """
        if self.parent_agent_type != None:
            raise RuntimeError(f"orbit data already loaded. It can only be assigned once.")            

        results_path_list = self.results_path.split('/')
        if 'results' in results_path_list[-1]:
            results_path_list.pop()

        scenario_name = results_path_list[-1]
        scenario_dir = f'./scenarios/{scenario_name}/'
        data_dir = scenario_dir + '/orbit_data/'

        with open(scenario_dir + '/MissionSpecs.json', 'r') as scenario_specs:
            # load json file as dictionary
            mission_dict : dict = json.load(scenario_specs)
            spacecraft_list : list = mission_dict.get('spacecraft', None)
            ground_station_list = mission_dict.get('groundStation', None)
            
            for spacecraft in spacecraft_list:
                spacecraft : dict
                name = spacecraft.get('name')
                index = spacecraft_list.index(spacecraft)
                agent_folder = "sat" + str(index) + '/'

                if name != self.get_parent_name():
                    continue

                # load eclipse data
                eclipse_file = data_dir + agent_folder + "eclipses.csv"
                eclipse_data = pd.read_csv(eclipse_file, skiprows=range(3))
                
                # load position data
                position_file = data_dir + agent_folder + "state_cartesian.csv"
                position_data = pd.read_csv(position_file, skiprows=range(4))

                # load propagation time data
                time_data =  pd.read_csv(position_file, nrows=3)
                _, epoc_type, _, epoc = time_data.at[0,time_data.axes[1][0]].split(' ')
                epoc_type = epoc_type[1 : -1]
                epoc = float(epoc)
                _, _, _, _, time_step = time_data.at[1,time_data.axes[1][0]].split(' ')
                time_step = float(time_step)

                time_data = { "epoc": epoc, 
                            "epoc type": epoc_type, 
                            "time step": time_step }

                # load inter-satellite link data
                isl_data = dict()
                for file in os.listdir(data_dir + '/comm/'):                
                    isl = re.sub(".csv", "", file)
                    sender, _, receiver = isl.split('_')

                    if 'sat' + str(index) in sender or 'sat' + str(index) in receiver:
                        isl_file = data_dir + 'comm/' + file
                        if 'sat' + str(index) in sender:
                            receiver_index = int(re.sub("[^0-9]", "", receiver))
                            receiver_name = spacecraft_list[receiver_index].get('name')
                            isl_data[receiver_name] = pd.read_csv(isl_file, skiprows=range(3))
                        else:
                            sender_index = int(re.sub("[^0-9]", "", sender))
                            sender_name = spacecraft_list[sender_index].get('name')
                            isl_data[sender_name] = pd.read_csv(isl_file, skiprows=range(3))

                # load ground station access data
                gs_access_data = pd.DataFrame(columns=['start index', 'end index', 'gndStn id', 'gndStn name','lat [deg]','lon [deg]'])
                for file in os.listdir(data_dir + agent_folder):
                    if 'gndStn' in file:
                        gndStn_access_file = data_dir + agent_folder + file
                        gndStn_access_data = pd.read_csv(gndStn_access_file, skiprows=range(3))
                        nrows, _ = gndStn_access_data.shape

                        if nrows > 0:
                            gndStn, _ = file.split('_')
                            gndStn_index = int(re.sub("[^0-9]", "", gndStn))
                            
                            gndStn_name = ground_station_list[gndStn_index].get('name')
                            gndStn_id = ground_station_list[gndStn_index].get('@id')
                            gndStn_lat = ground_station_list[gndStn_index].get('latitude')
                            gndStn_lon = ground_station_list[gndStn_index].get('longitude')

                            gndStn_name_column = [gndStn_name] * nrows
                            gndStn_id_column = [gndStn_id] * nrows
                            gndStn_lat_column = [gndStn_lat] * nrows
                            gndStn_lon_column = [gndStn_lon] * nrows

                            gndStn_access_data['gndStn name'] = gndStn_name_column
                            gndStn_access_data['gndStn id'] = gndStn_id_column
                            gndStn_access_data['lat [deg]'] = gndStn_lat_column
                            gndStn_access_data['lon [deg]'] = gndStn_lon_column

                            if len(gs_access_data) == 0:
                                gs_access_data = gndStn_access_data
                            else:
                                gs_access_data = pd.concat([gs_access_data, gndStn_access_data])

                # land coverage data metrics data
                payload = spacecraft.get('instrument', None)
                if not isinstance(payload, list):
                    payload = [payload]

                gp_access_data = pd.DataFrame(columns=['time index','GP index','pnt-opt index','lat [deg]','lon [deg]', 'agent','instrument',
                                                                'observation range [km]','look angle [deg]','incidence angle [deg]','solar zenith [deg]'])

                for instrument in payload:
                    i_ins = payload.index(instrument)
                    gp_acces_by_mode = []

                    # modes = spacecraft.get('instrument', None)
                    # if not isinstance(modes, list):
                    #     modes = [0]
                    modes = [0]

                    gp_acces_by_mode = pd.DataFrame(columns=['time index','GP index','pnt-opt index','lat [deg]','lon [deg]','instrument',
                                                                'observation range [km]','look angle [deg]','incidence angle [deg]','solar zenith [deg]'])
                    for mode in modes:
                        i_mode = modes.index(mode)
                        gp_access_by_grid = pd.DataFrame(columns=['time index','GP index','pnt-opt index','lat [deg]','lon [deg]',
                                                                'observation range [km]','look angle [deg]','incidence angle [deg]','solar zenith [deg]'])

                        for grid in mission_dict.get('grid'):
                            i_grid = mission_dict.get('grid').index(grid)
                            metrics_file = data_dir + agent_folder + f'datametrics_instru{i_ins}_mode{i_mode}_grid{i_grid}.csv'
                            metrics_data = pd.read_csv(metrics_file, skiprows=range(4))
                            
                            nrows, _ = metrics_data.shape
                            grid_id_column = [i_grid] * nrows
                            metrics_data['grid index'] = grid_id_column

                            if len(gp_access_by_grid) == 0:
                                gp_access_by_grid = metrics_data
                            else:
                                gp_access_by_grid = pd.concat([gp_access_by_grid, metrics_data])

                        nrows, _ = gp_access_by_grid.shape
                        gp_access_by_grid['pnt-opt index'] = [mode] * nrows

                        if len(gp_acces_by_mode) == 0:
                            gp_acces_by_mode = gp_access_by_grid
                        else:
                            gp_acces_by_mode = pd.concat([gp_acces_by_mode, gp_access_by_grid])
                        # gp_acces_by_mode.append(gp_access_by_grid)

                    nrows, _ = gp_acces_by_mode.shape
                    gp_access_by_grid['instrument'] = [instrument] * nrows
                    # gp_access_data[ins_name] = gp_acces_by_mode

                    if len(gp_access_data) == 0:
                        gp_access_data = gp_acces_by_mode
                    else:
                        gp_access_data = pd.concat([gp_access_data, gp_acces_by_mode])
                
                nrows, _ = gp_access_data.shape
                gp_access_data['agent name'] = [spacecraft['name']] * nrows

                grid_data_compiled = []
                for grid in mission_dict.get('grid'):
                    grid : dict
                    if grid.get('@type') == 'customGrid':
                        grid_file = grid.get('covGridFilePath')
                        # grid_data = pd.read_csv(grid_file)
                    elif grid.get('@type') == 'autogrid':
                        i_grid = mission_dict.get('grid').index(grid)
                        grid_file = data_dir + f'grid{i_grid}.csv'
                    else:
                        raise NotImplementedError(f"Loading of grids of type `{grid.get('@type')} not yet supported.`")

                    grid_data = pd.read_csv(grid_file)
                    nrows, _ = grid_data.shape
                    grid_data['GP index'] = [i for i in range(nrows)]
                    grid_data['grid index'] = [i_grid] * nrows
                    grid_data_compiled.append(grid_data)

                return OrbitData(name, time_data, eclipse_data, position_data, isl_data, gs_access_data, gp_access_data, grid_data_compiled)
                
    async def teardown(self) -> None:
        # log plan history
        headers = ['plan_index', 't_plan', 'req_id', 'subtask_index', 't_img', 'u_exp']
        data = []
        
        for i in range(len(self.plan_history)):
            t_plan, plan = self.plan_history[i]
            t_plan : float; plan : list

            for action in plan:
                if not isinstance(action, MeasurementAction):
                    continue

                req : MeasurementRequest = MeasurementRequest.from_dict(action.measurement_req)
                line_data = [   i,
                                np.round(t_plan,3),
                                req.id.split('-')[0],
                                action.subtask_index,
                                np.round(action.t_start,3 ),
                                np.round(action.u_exp,3)
                ]
                data.append(line_data)

        df = pd.DataFrame(data, columns=headers)
        self.log(f'\nPLANNER HISTORY\n{str(df)}\n', level=logging.WARNING)
        df.to_csv(f"{self.results_path}/{self.get_parent_name()}/planner_history.csv", index=False)

        # log performance stats
        n_decimals = 3
        headers = ['routine','t_avg','t_std','t_med','n']
        data = []

        for routine in self.stats:
            n = len(self.stats[routine])
            t_avg = np.round(np.mean(self.stats[routine]),n_decimals) if n > 0 else -1
            t_std = np.round(np.std(self.stats[routine]),n_decimals) if n > 0 else 0.0
            t_median = np.round(np.median(self.stats[routine]),n_decimals) if n > 0 else -1

            line_data = [ 
                            routine,
                            t_avg,
                            t_std,
                            t_median,
                            n
                            ]
            data.append(line_data)

        stats_df = pd.DataFrame(data, columns=headers)
        self.log(f'\nPLANNER RUN-TIME STATS\n{str(stats_df)}\n', level=logging.WARNING)
        stats_df.to_csv(f"{self.results_path}/{self.get_parent_name()}/planner_runtime_stats.csv", index=False)

        await super().teardown()