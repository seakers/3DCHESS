import copy
from typing import Union
from enum import Enum
from itertools import combinations, permutations
import numpy as np
import uuid
import numpy

class MeasurementRequetTypes(Enum):
    GROUND_POINT = 'GROUND_POINT'

class MeasurementRequest(object):
    """
    Describes a generic measurement request to be performed by agents in the simulation

    ### Attributes:
        - request_type (`str`): type of measurement request
        - s_max (`float`): maximum score attained from performing this task
        - measurements (`list`): measurement types required to perform this task
        - duration (`float`): duration of the measurement being performed
        - t_start (`float`): start time of the availability of this task in [s] from the beginning of the simulation
        - t_end (`float`): end time of the availability of this task in [s] from the beginning of the simulation
        - t_corr (`float`): maximum decorralation time between measurements of different measurements
        - id (`str`) : identifying number for this task in uuid format
    """        
    def __init__(self, 
                request_type : str,
                s_max : float,
                measurements : list,
                t_start: Union[float, int] = 0.0, 
                t_end: Union[float, int] = np.Inf, 
                t_corr: Union[float, int] = 0.0,
                duration: Union[float, int] = 0.0, 
                urgency: Union[float, int] = None,  
                id: str = None, 
                **_
                ) -> None:
        """
        Creates an instance of a measurement request 

        ### Arguments:
            - request_type (`str`): type of measurement request
            - s_max (`float`): maximum score attained from performing this task
            - measurements (`list`): measurement types required to perform this task
            - duration (`float`): duration of the measurement being performed
            - t_start (`float`): start time of the availability of this task in [s] from the beginning of the simulation
            - t_end (`float`): end time of the availability of this task in [s] from the beginning of the simulation
            - t_corr (`float`): maximum decorralation time between measurements of different measurements
            - id (`str`) : identifying number for this task in uuid format
        """
        # check arguments
        if not isinstance(s_max, float) and not isinstance(s_max, int):
            raise AttributeError(f'`s_max` must be of type `float` or type `int`. is of type {type(s_max)}.')
        if not isinstance(measurements, list):
            raise AttributeError(f'`instruments` must be of type `list`. is of type {type(measurements)}.')
        else:
            for measurement in measurements:
                if not isinstance(measurement, str):
                    raise AttributeError(f'`measurements` must a `list` of elements of type `str`. contains elements of type {type(measurement)}.')
        
        if t_start > t_end:
            raise ValueError(f"`t_start` must be smaller than `t_end`")
        if t_corr < 0:
            raise ValueError(f"`t_corr` must be non-negative.")
        
        # initialize
        self.request_type = request_type
        self.t_start = t_start
        self.t_end = t_end
        self.id = str(uuid.UUID(id)) if id is not None else str(uuid.uuid1())
        self.duration = duration
        self.s_max = s_max
        self.measurements = measurements    
        self.t_corr = t_corr
        if urgency is not None:
            self.urgency = urgency
        elif urgency == numpy.Inf:
            urgency = 0.0
        else:
            self.urgency = numpy.log(1e-3) / (t_start - t_end)
        
        self.measurement_groups = self.generate_measurement_groups(measurements)
        self.dependency_matrix = self.generate_dependency_matrix()
        self.time_dependency_matrix = self.generate_time_dependency_matrix()

    def generate_measurement_groups(self, measurements) -> list:
        """
        Generates all combinations of groups of measures to be performed by a single or multiple agents

        ### Arguments:
            - measurements (`list`): list of the measurements that are needed to fully perform this task

        ### Returns:
            - measurement_groups (`list`): list of measurement group tuples containing the main meausrement and a list of all dependent measurements
        """
        # create measurement groups
        n_measurements = len(measurements)
        measurement_groups = []
        for r in range(1, n_measurements+1):
            # combs = list(permutations(task_types, r))
            combs = list(combinations(measurements, r))
            
            for comb in combs:
                measurement_group = list(comb)

                main_measurement_permutations = list(permutations(comb, 1))
                for main_measurement in main_measurement_permutations:
                    main_measurement = list(main_measurement).pop()

                    dependend_measurements = copy.deepcopy(measurement_group)
                    dependend_measurements.remove(main_measurement)

                    if len(dependend_measurements) > 0:
                        measurement_groups.append((main_measurement, dependend_measurements))
                    else:
                        measurement_groups.append((main_measurement, []))
        
        return measurement_groups     
    
    def generate_dependency_matrix(self) -> list:
        # create dependency matrix
        dependency_matrix = []
        for index_a in range(len(self.measurement_groups)):
            main_a, dependents_a = self.measurement_groups[index_a]

            dependencies = []
            for index_b in range(len(self.measurement_groups)):
                main_b, dependents_b = self.measurement_groups[index_b]

                if index_a == index_b:
                    dependencies.append(0)

                elif main_a not in dependents_b or main_b not in dependents_a:
                    dependencies.append(-1)

                elif main_a == main_b:
                    dependencies.append(-1)
                    
                else:
                    dependents_a_extended : list = copy.deepcopy(dependents_a)
                    dependents_a_extended.remove(main_b)
                    dependents_b_extended : list = copy.deepcopy(dependents_b)
                    dependents_b_extended.remove(main_a)

                    if dependents_a_extended == dependents_b_extended:
                        dependencies.append(1)
                    else:
                        dependencies.append(-1)
            
            dependency_matrix.append(dependencies)
       
        return dependency_matrix

    def generate_time_dependency_matrix(self) -> list:
        time_dependency_matrix = []

        for index_a in range(len(self.measurement_groups)):
            time_dependencies = []
            for index_b in range(len(self.measurement_groups)):
                if self.dependency_matrix[index_a][index_b] > 0:
                    time_dependencies.append(self.t_corr)
                else:
                    time_dependencies.append(numpy.Inf)
            time_dependency_matrix.append(time_dependencies)

        return time_dependency_matrix

    def __repr__(self):
        task_id = self.id.split('-')
        return f'MeasurementReq_{task_id[0]}'

    def to_dict(self) -> dict:
        """
        Crates a dictionary containing all information contained in this measurement request object
        """
        return dict(self.__dict__)

    def from_dict(req : dict):
        if req['request_type'] == MeasurementRequetTypes.GROUND_POINT.value:
            return GroundPointMeasurementRequest(**req)
        else:
            raise NotImplementedError(f"Requests of type `{req['request_type']}` not yet supported.")

    def __eq__(self, other) -> bool:
        return self.to_dict() == other.to_dict()

    def pos_to_lat_lon(pos : list) -> list:
        R = 6.3781363e+003
        pos_vec = np.array(pos)

        r = np.sqrt( pos_vec.dot(pos_vec) )

        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])

        th1 = np.arccos( pos_vec.dot(z) / r ) * 180 / np.pi
        lat = 90 - th1

        x_proj = x * pos_vec.dot(x)
        y_proj = y * pos_vec.dot(y)
        lon = np.arctan2(y_proj, x_proj)

        alt = r - R

        return lat, lon, alt

    def lat_lon_to_pos(self, lat : float, lon : float, alt : float) -> list:
        R = 6.3781363e+003 + alt
        pos = [
                R * np.cos( lat * np.pi / 180.0) * np.cos( lon * np.pi / 180.0),
                R * np.cos( lat * np.pi / 180.0) * np.sin( lon * np.pi / 180.0),
                R * np.sin( lat * np.pi / 180.0)
        ] 
        return pos

    def copy(self) -> object:
        return MeasurementRequest.from_dict(self.to_dict())

class GroundPointMeasurementRequest(MeasurementRequest):
    """
    Describes a measurement reques of a specific Ground Point to be performed by agents in the simulation

    ### Attributes:
        - pos (`list`): lat-lon-alt coordinates of the location of this task
        - request_type (`str`): type of measurement request
        - s_max (`float`): maximum score attained from performing this task
        - measurements (`list`): measurement types required to perform this task
        - duration (`float`): duration of the measurement being performed
        - t_start (`float`): start time of the availability of this task in [s] from the beginning of the simulation
        - t_end (`float`): end time of the availability of this task in [s] from the beginning of the simulation
        - t_corr (`float`): maximum decorralation time between measurements of different measurements
        - id (`str`) : identifying number for this task in uuid format
    """        
    def __init__(self, 
                lat_lon_pos : list,
                s_max : float,
                measurements : list,
                t_start: Union[float, int], 
                t_end: Union[float, int], 
                t_corr: Union[float, int] = 0.0,
                duration: Union[float, int] = 0.0, 
                urgency: Union[float, int] = None,  
                pos : list = None,
                id: str = None, 
                **_
                ) -> None:
        """
        Creates an instance of a ground point measurement request 

        ### Arguments:
            - lat_lon_pos (`list`): lat-lon-alt coordinates of the location of this task [deg]
            - s_max (`float`): maximum score attained from performing this task
            - measurements (`list`): measurement types required to perform this task
            - duration (`float`): duration of the measurement being performed
            - t_start (`float`): start time of the availability of this task in [s] from the beginning of the simulation
            - t_end (`float`): end time of the availability of this task in [s] from the beginning of the simulation
            - t_corr (`float`): maximum decorralation time between measurements of different measurements
            - pos (`list`): cartesian coordinates of the location of this task [km]
            - id (`str`) : identifying number for this task in uuid format
        """
        if t_start == "inf" or t_start == "Inf" or t_start == "INF":
            t_start = np.Inf
        if t_end == "inf" or t_end == "Inf" or t_end == "INF":
            t_end = np.Inf
        if t_corr == "inf" or t_corr == "Inf" or t_corr == "INF":
            t_corr = np.Inf
        
        super().__init__(MeasurementRequetTypes.GROUND_POINT.value, 
                        s_max, 
                        measurements, 
                        t_start, 
                        t_end, 
                        t_corr, 
                        duration, 
                        urgency, 
                        id)
        
        if not isinstance(lat_lon_pos, list):
            raise AttributeError(f'`lat_lon_pos` must be of type `list`. is of type {type(lat_lon_pos)}.')
        elif len(lat_lon_pos) != 3:
            raise ValueError(f'`lat_lon_pos` must be a list of 3 values (lat, lon, alt). is of length {len(lat_lon_pos)}.')

        self.lat_lon_pos = lat_lon_pos
        lat, lon, alt = lat_lon_pos

        if pos is None:
            pos = self.lat_lon_to_pos(lat, lon, alt)
        if not isinstance(pos, list):
            raise AttributeError(f'`pos` must be of type `list`. is of type {type(pos)}.')
        elif len(pos) != 3:
            raise ValueError(f'`pos` must be a list of 3 values (x_pos, y_pos, z_pos). is of length {len(pos)}.')
        self.pos = pos