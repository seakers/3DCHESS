from abc import ABC

from typing import Union

from dmas.agents import AgentAction

# basilisk imports
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import FSW Algorithm related support
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport

class ComponentAction(ABC):
    """ 
    # Component Action
    
    Describes an action to be performed on or by an engineering module component
    """
    PENDING = 'PENDING'
    ABORTED = 'ABORTED'
    COMPLETED = 'COMPLETED'

    def __init__(   self, 
                    t_start : float = 0.0,
                    status : str = PENDING
                    ) -> None:
        self.t_start = t_start
        self.status = status

        super().__init__()

    pass

class SubsystemAction(ABC):
    """ 
    # Subsystem Action
    
    Describes an action to be performed on or by an engineering module subsystem
    """
    PENDING = 'PENDING'
    ABORTED = 'ABORTED'
    COMPLETED = 'COMPLETED'

    def __init__(   self, 
                    t_start : float = 0.0,
                    status : str = PENDING
                    ) -> None:
        self.t_start = t_start
        self.status = status

        super().__init__()
    pass

class SubsystemProvidePower(SubsystemAction):
    """ 
    # Subsystem Provide Power
    
    The action made by the Engineering Module to tell the EPS Subsystem to provide power to a component
    """
    def __init__(   self,
                    receiver : str,
                    t_start : float = 0.0,
                    status : str = SubsystemAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.receiver = receiver

class SubsystemStopProvidePower(SubsystemAction):
    """ 
    # Subsystem Provide Power
    
    The action made by the Engineering Module to tell the EPS Subsystem to provide power to a component
    """
    def __init__(   self,
                    receiver : str,
                    t_start : float = 0.0,
                    status : str = SubsystemAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.receiver = receiver

class ComponentProvidePower(ComponentAction):
    """ 
    # Component Provide Power
    
    The action made by the EPS Subsystem to tell a component to provide power to another component
    """
    def __init__(   self,
                    receiver_power : float,
                    t_start : float = 0.0,
                    status : str = ComponentAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.receiver_power = receiver_power

class ComponentStopProvidePower(ComponentAction):
    """ 
    # Component Stop Provide Power
    
    The action made by the EPS Subsystem to tell a component to provide power to another component
    """
    def __init__(   self,
                    receiver_power : float,
                    t_start : float = 0.0,
                    status : str = ComponentAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.receiver_power = receiver_power

class ComponentChargeBattery(ComponentAction):
    """ 
    # Component Charge Battery
    
    The action made by the EPS Subsystem to tell a solar panel to charge a battery
    """
    def __init__(   self,
                    charging_power : float,
                    t_start : float = 0.0,
                    status : str = ComponentAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.charging_power = charging_power

class ComponentStopChargeBattery(ComponentAction):
    """ 
    # Component Stop Charge Battery
    
    The action made by the EPS Subsystem to tell a solar panel to stop charging a battery
    """
    def __init__(   self,
                    charging_power : float,
                    t_start : float = 0.0,
                    status : str = ComponentAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.charging_power = charging_power

class RetrieveAtt(AgentAction):
    """ 
    # Subsystem Action
    
    Describes an action to be performed on or by an engineering module subsystem

    ### Attributes:
        - action_type (`str`): type of action to be performed
        - t_start (`float`): start time of this action in [s] from the beginning of the simulation
        - t_end (`float`): end time of this this action in [s] from the beginning of the simulation
        - status (`str`): completion status of the action
        - id (`str`) : identifying number for this action in uuid format
    """
    def __init__(self, 
                action_type: str,  
                t_start: Union[float, int], 
                status: str = 'PENDING', 
                id: str = None, 
                **_) -> None:
        super().__init__(action_type, t_start, status=status, id=id)

    def RetrieveAtt(self,
                    Inital_att : float,
                    dt : float,
            **kwargs) -> None:
        """
        """

        # Create simulation variable names
        simTaskName = "simTask"
        simProcessName = "simProcess"

        #  Create a sim module as an empty container
        scSim = SimulationBaseClass.SimBaseClass()

        # *******************set the simulation time ********************
        simulationTime = macros.sec2nano(self.dt) 

        # create the simulation process
        dynProcess = scSim.CreateNewProcess(simProcessName)
        # create the dynamics task and specify the integration update time
        simulationTimeStep = macros.sec2nano(0.1)
        dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

        # Setup the simulation tasks/objects

        # **********initialize spacecraft object and set properties**************
        scObject = spacecraft.Spacecraft()
        scObject.ModelTag = "bsk-Sat"
        # define the simulation inertia
        I = self.I_craft
        scObject.hub.mHub = self.mass# kg - spacecraft mass , self.mass
        scObject.hub.r_BcB_B = self.pos_vec  # m - position vector of body-fixed point B relative to CM, 
        scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

        # add spacecraft object to the simulation process
        scSim.AddModelToTask(simTaskName, scObject)

        # clear prior gravitational body and SPICE setup definitions
        gravFactory = simIncludeGravBody.gravBodyFactory()

        # setup Earth Gravity Body
        earth = gravFactory.createEarth()
        earth.isCentralBody = True  # ensure this is the central gravitational body
        mu = earth.mu

        # attach gravity model to spacecraft
        scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

        #
        #   initialize Spacecraft States with initialization variables
        #
        # **************setup the orbit using classical orbit elements**************
        oe = orbitalMotion.ClassicElements()
        # retrieve orbital elements from list [a,e,i,omega1,omega2,f]
        oe.a = self.Orb_elem[0]  # meters
        oe.e = self.Orb_elem[1]
        oe.i = self.Orb_elem[2] * macros.D2R
        oe.Omega = self.Orb_elem[3] * macros.D2R
        oe.omega = self.Orb_elem[4] * macros.D2R
        oe.f = self.Orb_elem[5] * macros.D2R
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        scObject.hub.sigma_BNInit = self.initial_att
        scObject.hub.omega_BN_BInit = self.ang_w

        


        # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
        # velocity navigation message
        sNavObject = simpleNav.SimpleNav()
        sNavObject.ModelTag = "SimpleNavigation"
        scSim.AddModelToTask(simTaskName, sNavObject)
        sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)


        # Setup data logging before the simulation is initialized
        numDataPoints = 1
        samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
        snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
        snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, snAttLog)
        scSim.AddModelToTask(simTaskName, snTransLog)

    
        # create simulation messages

        # if this scenario is to interface with the BSK Viz, uncomment the following lines
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                # , saveFile=fileName
                                                )

        # Initialize Simulation
        scSim.InitializeSimulation()

        # Configure a simulation stop time and execute the simulation run
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

        #
        #   retrieve the logged data
        #
        dataSigmaBN = snAttLog.sigma_BN
        return dataSigmaBN