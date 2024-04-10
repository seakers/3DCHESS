class Agent:
    def __init__(self, 
                 name : str, 
                 payload : list, 
                 bundle_size : float, 
                 t : float = 0.0) -> None:
        pass

    def propagate(self, percepts : list, t : float) -> list:
        """
        percepts (`list`): messages from other agents, measurement info
        """
        # sense - read percepts and classify depending on type
        bids = []
        tasks = []

        # think 
        actions = self.think(tasks, bids)
        
        return actions #TODO not final return

    def think(self, tasks : list, bids : list) -> list:
        # phase 1 - create a bundle 

        
        pass

if __name__ == '__main__':
    agent_1 = Agent('a1', ['VNIR'])

    tasks = []

    bids : list = agent_1.propagate(percepts=tasks, t=0.0)