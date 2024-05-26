import argparse
import csv
from datetime import datetime, timedelta
import json
import logging
from instrupy.base import Instrument
import orbitpy.util
import pandas as pd
import random
import zmq
import concurrent.futures

from dmas.messages import SimulationElementRoles
from dmas.network import NetworkConfig
from dmas.clocks import FixedTimesStepClockConfig, EventDrivenClockConfig
from src.nodes.planning.consensus.acbba import ACBBAReplanner
from src.nodes.planning.preplanners import *
from src.nodes.planning.replanners import *
from src.manager import SimulationManager
from src.monitor import ResultsMonitor

from src.nodes.states import *
from src.nodes.uav import UAVAgent
from src.nodes.agent import SimulationAgent
from src.nodes.groundstat import GroundStationAgent
from src.nodes.satellite import SatelliteAgent
from src.nodes.planning.planner import PlanningModule
from src.nodes.science.science import ScienceModule
from src.nodes.science.utility import utility_function
from src.nodes.science.reqs import GroundPointMeasurementRequest
from src.nodes.environment import SimulationEnvironment
from src.utils import *

# from satplan.visualizer import Visualizer

"""
======================================================
   _____ ____  ________  __________________
  |__  // __ \/ ____/ / / / ____/ ___/ ___/
   /_ </ / / / /   / /_/ / __/  \__ \\__ \ 
 ___/ / /_/ / /___/ __  / /___ ___/ /__/ / 
/____/_____/\____/_/ /_/_____//____/____/       (v1.0)
======================================================
                Texas A&M - SEAK Lab
======================================================

Wrapper for running DMAS simulations for the 3DCHESS project
"""
def print_welcome(scenario_name) -> None:
    os.system('cls' if os.name == 'nt' else 'clear')
    out = "\n======================================================"
    out += '\n   _____ ____        ________  __________________\n  |__  // __ \      / ____/ / / / ____/ ___/ ___/\n   /_ </ / / /_____/ /   / /_/ / __/  \__ \\__ \ \n ___/ / /_/ /_____/ /___/ __  / /___ ___/ /__/ / \n/____/_____/      \____/_/ /_/_____//____/____/ (v1.0)'
    out += "\n======================================================"
    out += '\n\tTexas A&M University - SEAK Lab ©'
    out += "\n======================================================"
    out += f"\nSCENARIO: {scenario_name}"
    print(out)

def main(   scenario_name : str, 
            scenario_path : str,
            plot_results : bool = False, 
            save_plot : bool = False, 
            level : int = logging.WARNING
        ) -> None:
    """
    Runs Simulation 
    """
    # create results directory
    results_path = setup_results_directory(scenario_path)

    # select unsused ports
    port = random.randint(5555, 9999)
    
    # load scenario json file
    scenario_filename = os.path.join(scenario_path, 'MissionSpecs.json')
    with open(scenario_filename, 'r') as scenario_file:
        scenario_dict : dict = json.load(scenario_file)

    # unpack agent info
    spacecraft_dict = scenario_dict.get('spacecraft', None)
    uav_dict        = scenario_dict.get('uav', None)
    gstation_dict   = scenario_dict.get('groundStation', None)
    settings_dict   = scenario_dict.get('settings', None)

    # read agent names
    agent_names = [SimulationElementRoles.ENVIRONMENT.value]
    if spacecraft_dict:
        agent_names.extend([spacecraft['name'] for spacecraft in spacecraft_dict])
    if uav_dict:
        agent_names.extend([uav['name'] for uav in uav_dict])
    if gstation_dict:
        agent_names.extend([gstation['name'] for gstation in gstation_dict])

    # read logger level
    if isinstance(settings_dict, dict):
        level = settings_dict.get('logger', logging.WARNING)
        if not isinstance(level, int):
            levels = {
                        'DEBUG': logging.DEBUG, 
                        'WARNING' : logging.WARNING,
                        'CRITICAL' : logging.CRITICAL,
                        'ERROR' : logging.ERROR
                    }
            level = levels[level]
    else:
        level = logging.WARNING

    # precompute orbit data
    orbitdata_dir = precompute_orbitdata(scenario_name) if spacecraft_dict is not None else None

    # read clock configuration
    epoch_dict : dict = scenario_dict.get("epoch")
    year = epoch_dict.get('year', None)
    month = epoch_dict.get('month', None)
    day = epoch_dict.get('day', None)
    hh = epoch_dict.get('hour', None)
    mm = epoch_dict.get('minute', None)
    ss = epoch_dict.get('second', None)
    start_date = datetime(year, month, day, hh, mm, ss)
    delta = timedelta(days=scenario_dict.get("duration"))
    end_date = start_date + delta

    ## define simulation clock
    if spacecraft_dict:
        for spacecraft in spacecraft_dict:
            spacecraft_dict : list
            spacecraft : dict
            index = spacecraft_dict.index(spacecraft)
            agent_dir = f"sat{str(index)}"

            position_file = os.path.join(orbitdata_dir, agent_dir, 'state_cartesian.csv')
            time_data =  pd.read_csv(position_file, nrows=3)
            l : str = time_data.at[1,time_data.axes[1][0]]
            _, _, _, _, dt = l.split(' ')
            dt = float(dt)
            break
    else:
        dt = delta.total_seconds()/100

    ## load initial measurement request
    measurement_reqs = load_measurement_reqs(scenario_dict, spacecraft_dict, uav_dict, delta)

    # clock_config = FixedTimesStepClockConfig(start_date, end_date, dt)
    clock_config = EventDrivenClockConfig(start_date, end_date)

    # initialize manager
    manager_network_config = NetworkConfig( scenario_name,
											manager_address_map = {
																	zmq.REP: [f'tcp://*:{port}'],
																	zmq.PUB: [f'tcp://*:{port+1}'],
                                                                    zmq.SUB: [f'tcp://*:{port+2}'],
																	zmq.PUSH: [f'tcp://localhost:{port+3}']
                                                                    }
                                            )


    manager = SimulationManager(results_path, agent_names, clock_config, manager_network_config, level)
    logger = manager.get_logger()

    # create results monitor
    monitor_network_config = NetworkConfig( scenario_name,
                                    external_address_map = {zmq.SUB: [f'tcp://localhost:{port+1}'],
                                                            zmq.PULL: [f'tcp://*:{port+3}']}
                                    )
    
    monitor = ResultsMonitor(clock_config, monitor_network_config, logger=logger)

    # # unpack scenario
    scenario_config_dict : dict = scenario_dict['scenario']
    events_path = scenario_config_dict.get('eventsPath', None)
    
    # Create agents 
    agents = []
    agent_port = port + 6
    if spacecraft_dict is not None:
        for d in spacecraft_dict:
            # Create spacecraft agents
            agent = agent_factory(  scenario_name, 
                                    scenario_path, 
                                    results_path, 
                                    orbitdata_dir, 
                                    d,
                                    spacecraft_dict.index(d), 
                                    manager_network_config, 
                                    agent_port, 
                                    SimulationAgentTypes.SATELLITE, 
                                    clock_config, 
                                    logger,
                                    measurement_reqs,
                                    events_path,
                                    delta
                                )
            agents.append(agent)
            agent_port += 6

    if uav_dict is not None:
        # Create uav agents
        for d in uav_dict:
            agent = agent_factory(  scenario_name, 
                                    scenario_path, 
                                    results_path, 
                                    orbitdata_dir, 
                                    d, 
                                    uav_dict.index(d),
                                    manager_network_config, 
                                    agent_port, 
                                    SimulationAgentTypes.UAV, 
                                    clock_config, 
                                    logger,
                                    measurement_reqs,
                                    events_path,
                                    delta
                                )
            agents.append(agent)
            agent_port += 6

    if gstation_dict is not None:
        # TODO Create ground station agents
        raise NotImplementedError('Ground Station agents not yet implemented.')
        # for d in gstation_dict:
        #     pass
    #         d : dict
    #         agent_name = d['name']
    #         lat = d['latitude']
    #         lon = d['longitude']
    #         alt = d['altitude']
    #         initial_state = GroundStationAgentState(lat,
    #                                                 lon,
    #                                                 alt)

    #         agent = GroundStationAgent( agent_name, 
    #                                     results_path,
    #                                     scenario_name,
    #                                     agent_port,
    #                                     manager_network_config,
    #                                     initial_state,
    #                                     utility_function[env_utility_function],
    #                                     initial_reqs=measurement_reqs,
    #                                     logger=logger)
    #         agents.append(agent)
    #         agent_port += 6

    # create environment
    ## unpack config
    env_utility_function = scenario_config_dict.get('utility', 'LINEAR')
    env_connectivity = scenario_config_dict.get('connectivity', 'FULL')
    
    ## subscribe to all elements in the network
    env_subs = []
    for agent in agents:
        agent_pubs : str = agent._network_config.external_address_map[zmq.PUB]
        for agent_pub in agent_pubs:
            env_sub : str = agent_pub.replace('*', 'localhost')
            env_subs.append(env_sub)
    
    ## create network config
    env_network_config = NetworkConfig( manager.get_network_config().network_name,
											manager_address_map = {
													zmq.REQ: [f'tcp://localhost:{port}'],
													zmq.SUB: [f'tcp://localhost:{port+1}'],
                                                    zmq.PUB: [f'tcp://localhost:{port+2}'],
													zmq.PUSH: [f'tcp://localhost:{port+3}']},
											external_address_map = {
													zmq.REP: [f'tcp://*:{port+4}'],
													zmq.PUB: [f'tcp://*:{port+5}'],
                                                    zmq.SUB: env_subs
											})
    
    ## initialize environment
    environment = SimulationEnvironment(scenario_path, 
                                        results_path, 
                                        env_network_config, 
                                        manager_network_config,
                                        utility_function[env_utility_function],
                                        measurement_reqs, 
                                        events_path,
                                        logger=logger)
            
    # run simulation
    with concurrent.futures.ThreadPoolExecutor(len(agents) + 3) as pool:
        pool.submit(monitor.run, *[])
        pool.submit(manager.run, *[])
        pool.submit(environment.run, *[])
        for agent in agents:                
            agent : SimulationAgent
            pool.submit(agent.run, *[])    

    # convert outputs for satplan visualizer
    measurements_path = results_path + '/' + environment.get_element_name().lower() + '/measurements.csv'
    performed_measurements : pd.DataFrame = pd.read_csv(measurements_path)

    spacecraft_names = [spacecraft['name'] for spacecraft in spacecraft_dict]
    spacecraft_ids = os.listdir(orbitdata_dir)

    for spacecraft in spacecraft_dict:
        spacecraft : dict
        name = spacecraft.get('name')
        index = spacecraft_dict.index(spacecraft)
        spacecraft_id =  "sat" + str(index)
        plan_path = orbitdata_dir+'/'+spacecraft_id+'/plan.csv'

        with open(plan_path,'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
            
            sat_measurements : pd.DataFrame = performed_measurements.query('`measurer` == @name').sort_values(by=['t_img'])
            for _, row in sat_measurements.iterrows():
                lat,lon = None, None
                for measurement_req in measurement_reqs:
                    measurement_req : GroundPointMeasurementRequest
                    if row['req_id'] in measurement_req.id:
                        lat,lon,_ = measurement_req.lat_lon_pos
                
                if lat is None and lon is None:
                    continue

                obs = {
                    "start" : row['t_img'],
                    "end" : row['t_img'] + dt,
                    "location" : {
                                    "lat" : lat,
                                    "lon" : lon
                                    }
                }
                row_out = [obs["start"],obs["end"],obs["location"]["lat"],obs["location"]["lon"]]
                csvwriter.writerow(row_out)

    if plot_results:
        raise NotImplementedError("Visualization integration with `satplan` not yet supported.")
        
        visualizer = Visualizer(scenario_path+'/',
                                results_path+'/',
                                start_date,
                                dt,
                                scenario_dict.get("duration"),
                                orbitdata_dir+'/grid0.csv'
                                )
        visualizer.process_mission_data()
        visualizer.plot_mission()
        
        if save_plot:
            raise NotImplementedError("Saving of `satplan` animations not yet supported.")
    
    print(f'\nSIMULATION FOR SCENARIO `{scenario_name}` DONE')

def setup_results_directory(scenario_path) -> str:
    """
    Creates an empty results directory within the current working directory
    """
    results_path = f'{scenario_path}' if '/results/' in scenario_path else os.path.join(scenario_path, 'results')

    if not os.path.exists(results_path):
        # create results directory if it doesn't exist
        os.makedirs(results_path)
    else:
        # clear results in case it already exists
        results_path
        for filename in os.listdir(results_path):
            file_path = os.path.join(results_path, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print('Failed to delete %s. Reason: %s' % (file_path, e))

    return results_path

def load_measurement_reqs(scenario_dict : dict, spacecraft_dict : dict, uav_dict : dict, delta : timedelta) -> list:
    measurement_reqs = []
    if scenario_dict['scenario']['@type'] == 'RANDOM':
        reqs_dict = scenario_dict['scenario']['requests']
        for i_req in range(reqs_dict['n']):
            if spacecraft_dict:
                raise NotImplementedError('random spacecraft scenario not yet implemented.')

            elif uav_dict: 
                max_distance = np.sqrt((reqs_dict['x_bounds'][1] - reqs_dict['x_bounds'][0]) **2 + (reqs_dict['y_bounds'][1] - reqs_dict['y_bounds'][0])**2)
                x_pos = reqs_dict['x_bounds'][0] + (reqs_dict['x_bounds'][1] - reqs_dict['x_bounds'][0]) * random.random()
                y_pos = reqs_dict['y_bounds'][0] + (reqs_dict['y_bounds'][1] - reqs_dict['y_bounds'][0]) * random.random()
                z_pos = 0.0
                pos = [x_pos, y_pos, z_pos]
                lan_lon_pos = [0.0, 0.0, 0.0]

            s_max = 100
            measurements = []
            required_measurements = reqs_dict['measurement_reqs']
            for _ in range(random.randint(1, len(required_measurements))):
                measurement = random.choice(required_measurements)
                while measurement in measurements:
                    measurement = random.choice(required_measurements)
                measurements.append(measurement)


            # t_start = delta.seconds * random.random()
            t_start = 0.0
            # t_end = t_start + (delta.seconds - t_start) * random.random()
            # t_end = t_end if t_end - t_start > max_distance else t_start + max_distance
            t_end = delta.seconds
            t_corr = t_end - t_start

            req = GroundPointMeasurementRequest(lan_lon_pos, s_max, measurements, t_start, t_end, t_corr, pos=pos)
            measurement_reqs.append(req)

    else:
        initialRequestsPath = scenario_dict['scenario'].get('initialRequestsPath', scenario_path + '/gpRequests.csv')
        df = pd.read_csv(initialRequestsPath)
            
        for _, row in df.iterrows():
            s_max = row.get('severity',row.get('s_max', None))
            
            measurements_str : str = row['measurements']
            measurements_str = measurements_str.replace('[','')         # remove left bracket
            measurements_str = measurements_str.replace(']','')         # remove right bracket
            measurements_str = measurements_str.replace(', ',',')       # remove spaces
            measurements_str = measurements_str.replace('\'','')        # remove quotes if any
            measurements = measurements_str.split(',')                  # plit into measurements

            t_start = row.get('start time [s]',row.get('t_start', None))
            t_end =  row.get('t_end', t_start + row.get('duration [s]', None) )
            t_corr = row.get('t_corr',t_end - t_start)

            lat = row.get('lat', row.get('lat [deg]', None)) 
            lon = row.get('lon', row.get('lon [deg]', None))
            alt = row.get('alt', 0.0)
            if lat is None or lon is None or alt is None: 
                x_pos, y_pos, z_pos = row.get('x_pos', None), row.get('y_pos', None), row.get('z_pos', None)
                if x_pos is not None and y_pos is not None and z_pos is not None:
                    pos = [x_pos, y_pos, z_pos]
                    lat, lon, alt = 0.0, 0.0, 0.0
                else:
                    raise ValueError('GP Measurement Requests in `gpRequest.csv` must specify a ground position as lat-lon-alt or cartesian coordinates.')
            else:
                pos = None

            lan_lon_pos = [lat, lon, alt]
            req = GroundPointMeasurementRequest(lan_lon_pos, s_max, measurements, t_start, t_end, t_corr, pos=pos)
            measurement_reqs.append(req)

    return measurement_reqs

def precompute_orbitdata(scenario_name) -> str:
    """
    Pre-calculates coverage and position data for a given scenario
    """
    
    scenario_dir = scenario_name if './scenarios/' in scenario_name else os.path.join('./scenarios', scenario_name)
    if './scenarios/' in scenario_name and '/orbit_data/' in scenario_name:
        data_dir = scenario_name
    else:
        data_dir = os.path.join('./scenarios/', scenario_name, 'orbit_data')
   
    if not os.path.exists(data_dir):
        # if directory does not exists, create it
        os.mkdir(data_dir)
        changes_to_scenario = True
    else:
        changes_to_scenario : bool = check_changes_to_scenario(scenario_dir, data_dir)

    if not changes_to_scenario:
        # if propagation data files already exist, load results
        print('Orbit data found!')
    else:
        # if propagation data files do not exist, propagate and then load results
        if os.path.exists(data_dir):
            print('Existing orbit data does not match scenario.')
        else:
            print('Orbit data not found.')

        # clear files if they exist
        print('Clearing \'orbitdata\' directory...')    
        if os.path.exists(data_dir):
            for f in os.listdir(data_dir):
                f_dir = os.path.join(data_dir, f)
                if os.path.isdir(f_dir):
                    for h in os.listdir(f_dir):
                        os.remove(os.path.join(f_dir, h))
                    os.rmdir(f_dir)
                else:
                    os.remove(f_dir) 
        print('\'orbitdata\' cleared!')

        scenario_file = os.path.join(scenario_dir, 'MissionSpecs.json') 
        with open(scenario_file, 'r') as scenario_specs:
            # load json file as dictionary
            mission_dict : dict = json.load(scenario_specs)

            # set grid 
            grid_dicts : list = mission_dict.get("grid", None)
            for grid_dict in grid_dicts:
                grid_dict : dict
                if grid_dict is not None:
                    grid_type : str = grid_dict.get('@type', None)
                    
                    if grid_type.lower() == 'customgrid':
                        # do nothing
                        pass
                    elif grid_type.lower() == 'uniform':
                        # create uniform grid
                        lat_spacing = grid_dict.get('lat_spacing', 1)
                        lon_spacing = grid_dict.get('lon_spacing', 1)
                        grid_index  = grid_dicts.index(grid_dict)
                        grid_path : str = create_uniform_grid(scenario_dir, grid_index, lat_spacing, lon_spacing)

                        # set to customgrid
                        grid_dict['@type'] = 'customgrid'
                        grid_dict['covGridFilePath'] = grid_path
                        
                    elif grid_type.lower() == 'cluster':
                        # create clustered grid
                        n_clusters          = grid_dict.get('n_clusters', 100)
                        n_cluster_points    = grid_dict.get('n_cluster_points', 1)
                        variance            = grid_dict.get('variance', 1)
                        grid_index          = grid_dicts.index(grid_dict)
                        grid_path : str = create_clustered_grid(scenario_dir, grid_index, n_clusters, n_cluster_points, variance)

                        # set to customgrid
                        grid_dict['@type'] = 'customgrid'
                        grid_dict['covGridFilePath'] = grid_path
                        
                    else:
                        raise ValueError(f'Grids of type \'{grid_type}\' not supported.')
                else:
                    pass
            mission_dict['grid'] = grid_dicts

            # set output directory to orbit data directory
            if mission_dict.get("settings", None) is not None:
                mission_dict["settings"]["outDir"] = scenario_dir + '/orbit_data/'
            else:
                mission_dict["settings"] = {}
                mission_dict["settings"]["outDir"] = scenario_dir + '/orbit_data/'

            # propagate data and save to orbit data directory
            print("Propagating orbits...")
            mission : Mission = Mission.from_json(mission_dict)  
            mission.execute()                
            print("Propagation done!")

            # save specifications of propagation in the orbit data directory
            with open(os.path.join(data_dir,'MissionSpecs.json'), 'w') as mission_specs:
                mission_specs.write(json.dumps(mission_dict, indent=4))

    return data_dir

def create_uniform_grid(scenario_dir : str, grid_index : int, lat_spacing : float, lon_spacing : float) -> str:
    # create uniform grid
    groundpoints = [(lat, lon) 
                    for lat in numpy.linspace(-90, 90, int(180/lat_spacing)+1)
                    for lon in numpy.linspace(-180, 180, int(360/lon_spacing)+1)
                    if lon < 180
                    ]
    
    # create datagrame
    df = pd.DataFrame(data=groundpoints, columns=['lat [deg]','lon [deg]'])

    # save to csv
    grid_path : str = os.path.join(scenario_dir, 'resources', f'uniform_grid{grid_index}.csv')
    df.to_csv(grid_path,index=False)

    # return address
    return grid_path

def create_clustered_grid(scenario_dir : str, grid_index : int, n_clusters : int, n_cluster_points : int, variance : float) -> str:
    # create clustered grid of gound points
    std = numpy.sqrt(variance)
    groundpoints = []
    
    for _ in range(n_clusters):
        # find cluster center
        lat_cluster = (90 - -90) * random.random() -90
        lon_cluster = (180 - -180) * random.random() -180
        
        for _ in range(n_cluster_points):
            # sample groundpoint
            lat = random.normalvariate(lat_cluster, std)
            lon = random.normalvariate(lon_cluster, std)
            groundpoints.append((lat,lon))

    # create datagrame
    df = pd.DataFrame(data=groundpoints, columns=['lat [deg]','lon [deg]'])

    # save to csv
    grid_path : str = os.path.join(scenario_dir, 'resources', f'cluster_grid{grid_index}.csv')
    df.to_csv(grid_path,index=False)

    # return address
    return grid_path

def check_changes_to_scenario(scenario_dir : str, orbitdata_dir : str) -> bool:
    """ 
    Checks if the scenario has already been pre-computed 
    or if relevant changes have been made 
    """

    filename = 'MissionSpecs.json'
    scenario_filename = os.path.join(scenario_dir, filename)

    with open(scenario_filename, 'r') as scenario_specs:
        # check if data has been previously calculated
        orbitdata_filename = os.path.join(orbitdata_dir, filename)
        if not os.path.exists(orbitdata_filename):
            return True
            
        with open(orbitdata_filename, 'r') as orbitdata_specs:
            scenario_dict : dict = json.load(scenario_specs)
            orbitdata_dict : dict = json.load(orbitdata_specs)

            scenario_dict.pop('settings')
            orbitdata_dict.pop('settings')
            scenario_dict.pop('scenario')
            orbitdata_dict.pop('scenario')

            if (
                    scenario_dict['epoch'] != orbitdata_dict['epoch']
                or scenario_dict['duration'] != orbitdata_dict['duration']
                or scenario_dict.get('groundStation', None) != orbitdata_dict.get('groundStation', None)
                # or scenario_dict['grid'] != orbitdata_dict['grid']
                # or scenario_dict['scenario']['connectivity'] != mission_dict['scenario']['connectivity']
                ):
                return True
            
            if scenario_dict['grid'] != orbitdata_dict['grid']:
                if len(scenario_dict['grid']) != len(orbitdata_dict['grid']):
                    return True
                
                for i in range(len(scenario_dict['grid'])):
                    scenario_grid : dict = scenario_dict['grid'][i]
                    mission_grid : dict = orbitdata_dict['grid'][i]

                    scenario_gridtype = scenario_grid['@type'].lower()
                    mission_gridtype = mission_grid['@type'].lower()

                    # if  != 'customgrid'
                    if scenario_gridtype != mission_gridtype == 'customgrid':
                        if scenario_gridtype not in mission_grid['covGridFilePath']:
                            return True

            if scenario_dict['spacecraft'] != orbitdata_dict['spacecraft']:
                if len(scenario_dict['spacecraft']) != len(orbitdata_dict['spacecraft']):
                    return True
                
                for i in range(len(scenario_dict['spacecraft'])):
                    scenario_sat : dict = scenario_dict['spacecraft'][i]
                    mission_sat : dict = orbitdata_dict['spacecraft'][i]
                    
                    if "planner" in scenario_sat:
                        scenario_sat.pop("planner")
                    if "science" in scenario_sat:
                        scenario_sat.pop("science")
                    if "notifier" in scenario_sat:
                        scenario_sat.pop("notifier") 
                    if "missionProfile" in scenario_sat:
                        scenario_sat.pop("missionProfile")

                    if "planner" in mission_sat:
                        mission_sat.pop("planner")
                    if "science" in mission_sat:
                        mission_sat.pop("science")
                    if "notifier" in mission_sat:
                        mission_sat.pop("notifier") 
                    if "missionProfile" in mission_sat:
                        mission_sat.pop("missionProfile")

                    if scenario_sat != mission_sat:
                        return True
                    
    return False

def agent_factory(  scenario_name : str, 
                    scenario_path : str,
                    results_path : str,
                    orbitdata_dir : str,
                    agent_dict : dict, 
                    agent_index : int,
                    manager_network_config : NetworkConfig, 
                    port : int, 
                    agent_type : SimulationAgentTypes,
                    clock_config : float,
                    logger : logging.Logger,
                    initial_reqs : list,
                    events_path : str,
                    delta : timedelta
                ) -> SimulationAgent:
    ## unpack mission specs
    agent_name = agent_dict['name']
    planner_dict = agent_dict.get('planner', None)
    science_dict = agent_dict.get('science', None)
    instruments_dict = agent_dict.get('instrument', None)
    orbit_state_dict = agent_dict.get('orbitState', None)

    ## create agent network config
    manager_addresses : dict = manager_network_config.get_manager_addresses()
    req_address : str = manager_addresses.get(zmq.REP)[0]
    req_address = req_address.replace('*', 'localhost')

    sub_address : str = manager_addresses.get(zmq.PUB)[0]
    sub_address = sub_address.replace('*', 'localhost')

    pub_address : str = manager_addresses.get(zmq.SUB)[0]
    pub_address = pub_address.replace('*', 'localhost')

    push_address : str = manager_addresses.get(zmq.PUSH)[0]

    agent_network_config = NetworkConfig( 	scenario_name,
                                            manager_address_map = {
                                                    zmq.REQ: [req_address],
                                                    zmq.SUB: [sub_address],
                                                    zmq.PUB: [pub_address],
                                                    zmq.PUSH: [push_address]},
                                            external_address_map = {
                                                    zmq.REQ: [],
                                                    zmq.SUB: [f'tcp://localhost:{port+1}'],
                                                    zmq.PUB: [f'tcp://*:{port+2}']},
                                            internal_address_map = {
                                                    zmq.REP: [f'tcp://*:{port+3}'],
                                                    zmq.PUB: [f'tcp://*:{port+4}'],
                                                    zmq.SUB: [  
                                                                f'tcp://localhost:{port+5}',
                                                                f'tcp://localhost:{port+6}'
                                                            ]
                                        })

    ## load payload
    if instruments_dict:
        payload = orbitpy.util.dictionary_list_to_object_list(instruments_dict, Instrument) # list of instruments
    else:
        payload = []

    ## load science module
    if science_dict is not None and science_dict == "True":
        science = ScienceModule(results_path,scenario_path,agent_name,agent_network_config,events_path,logger=logger)
    else:
        science = None
        # raise NotImplementedError(f"Science module not yet implemented.")

    ## load planner module
    if planner_dict is not None:
        planner_dict : dict
        utility_func = utility_function[planner_dict.get('utility', 'NONE')]
        
        preplanner_dict = planner_dict.get('preplanner', None)
        if isinstance(preplanner_dict, dict):
            preplanner_type = preplanner_dict.get('@type', None)
            period = preplanner_dict.get('period', np.Inf)
            horizon = preplanner_dict.get('horizon', np.Inf)

            if preplanner_type == "FIFO":
                collaboration = preplanner_dict.get('collaboration', "False") == "True"
                preplanner = FIFOPreplanner(utility_func, period, horizon, collaboration)
            else:
                raise NotImplementedError(f'preplanner of type `{preplanner_dict}` not yet supported.')
        else:
            preplanner = None

        replanner_dict = planner_dict.get('replanner', None)
        if isinstance(replanner_dict, dict):
            replanner_type = replanner_dict.get('@type', None)
            
            if replanner_type == 'FIFO':
                collaboration = replanner_dict.get('collaboration', "False") == "True"
                replanner = FIFOReplanner(utility_func, collaboration)

            elif replanner_type == 'ACBBA':
                max_bundle_size = replanner_dict.get('bundle size', 3)
                dt_converge = replanner_dict.get('dt_convergence', 0.0)
                period = replanner_dict.get('period', 60.0)
                threshold = replanner_dict.get('threshold', 1)
                horizon = replanner_dict.get('horizon', delta.total_seconds())

                replanner = ACBBAReplanner(utility_func, max_bundle_size, dt_converge, period, threshold, horizon)
            
            else:
                raise NotImplementedError(f'replanner of type `{replanner_dict}` not yet supported.')
        else:
            # replanner = None
            replanner = RelayReplanner()
    else:
        preplanner, replanner, utility_func = None, None, 'NONE'

    planner = PlanningModule(   results_path, 
                                agent_name, 
                                agent_network_config, 
                                utility_func, 
                                preplanner,
                                replanner,
                                initial_reqs
                            )    
        
    ## create agent
    if agent_type == SimulationAgentTypes.UAV:
        ## load initial state 
            pos = agent_dict['pos']
            max_speed = agent_dict['max_speed']
            if isinstance(clock_config, FixedTimesStepClockConfig):
                eps = max_speed * clock_config.dt / 2.0
            else:
                eps = 1e-6

            initial_state = UAVAgentState(  agent_name,
                                            [instrument.name for instrument in payload], 
                                            pos, 
                                            max_speed, 
                                            eps=eps )

            ## create agent
            return UAVAgent(   agent_name, 
                                results_path,
                                manager_network_config,
                                agent_network_config,
                                initial_state,
                                payload,
                                planner,
                                science,
                                logger=logger
                            )

    elif agent_type == SimulationAgentTypes.SATELLITE:
        agent_folder = "sat" + str(agent_index) + '/'

        position_file = orbitdata_dir + agent_folder + 'state_cartesian.csv'
        time_data =  pd.read_csv(position_file, nrows=3)
        l : str = time_data.at[1,time_data.axes[1][0]]
        _, _, _, _, dt = l.split(' '); dt = float(dt)

        initial_state = SatelliteAgentState(agent_name,
                                            orbit_state_dict, 
                                            [instrument.name for instrument in payload], 
                                            time_step=dt) 
        
        return SatelliteAgent(
                                agent_name,
                                results_path,
                                manager_network_config,
                                agent_network_config,
                                initial_state, 
                                planner,
                                payload,
                                science,
                                logger=logger
                            )
    else:
        raise NotImplementedError(f"agents of type `{agent_type}` not yet supported by agent factory.")

if __name__ == "__main__":
    
    # read system arguments
    parser = argparse.ArgumentParser(
                    prog='DMAS for 3D-CHESS',
                    description='Simulates an autonomous Earth-Observing satellite mission.',
                    epilog='- TAMU')

    parser.add_argument(    'scenario_name', 
                            help='name of the scenario being simulated',
                            type=str)
    parser.add_argument(    '-p', 
                            '--plot-result',
                            action='store_true',
                            help='creates animated plot of the simulation',
                            required=False,
                            default=False)    
    parser.add_argument(    '-s', 
                            '--save-plot',
                            action='store_true',
                            help='saves animated plot of the simulation as a gif',
                            required=False,
                            default=False) 
    parser.add_argument(    '-d', 
                            '--no-graphic',
                            action='store_true',
                            help='does not draws ascii welcome screen graphic',
                            required=False,
                            default=False)  
    parser.add_argument(    '-l', 
                            '--level',
                            choices=['DEBUG', 'INFO', 'WARNING', 'CRITICAL', 'ERROR'],
                            default='WARNING',
                            help='logging level',
                            required=False,
                            type=str)  
                    
    args = parser.parse_args()
    
    scenario_name = args.scenario_name
    plot_results = args.plot_result
    save_plot = args.save_plot
    no_grapgic = args.no_graphic

    levels = {  'DEBUG' : logging.DEBUG, 
                'INFO' : logging.INFO, 
                'WARNING' : logging.WARNING, 
                'CRITICAL' : logging.CRITICAL, 
                'ERROR' : logging.ERROR
            }
    level = levels.get(args.level)

    # terminal welcome message
    if not no_grapgic:
        print_welcome(scenario_name)
    
    # load scenario json file
    scenario_path = f"{scenario_name}" if "./scenarios/" in scenario_name else f'./scenarios/{scenario_name}/'
    scenario_file = open(scenario_path + '/MissionSpecs.json', 'r')
    scenario_dict : dict = json.load(scenario_file)
    scenario_file.close()

    main(scenario_name, scenario_path, plot_results, save_plot, levels)
    