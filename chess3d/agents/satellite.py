import logging

from dmas.network import NetworkConfig

from chess3d.agents.science.module import ScienceModule
from chess3d.agents.planning.module import PlanningModule
from chess3d.agents.agent import SimulationAgentState
from chess3d.agents.agent import SimulationAgent

class SatelliteAgent(SimulationAgent):
    def __init__(   self, 
                    agent_name: str, 
                    results_path: str, 
                    manager_network_config: NetworkConfig, 
                    agent_network_config: NetworkConfig,
                    initial_state: SimulationAgentState, 
                    planning_module : PlanningModule,
                    payload: list, 
                    science_module: ScienceModule = None, 
                    level: int = logging.INFO, 
                    logger: logging.Logger = None
                    ) -> None:

        
        super().__init__(agent_name, 
                        results_path, 
                        manager_network_config, 
                        agent_network_config, 
                        initial_state, 
                        payload, 
                        planning_module, 
                        science_module, 
                        level, 
                        logger)

    async def setup(self) -> None:
        # nothing to setup
        return