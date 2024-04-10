# TODO - create wrapper

class Wrapper:
    """
    Goal: to have every agent run their sense->think->do loop at each time-step 
    and compile and classify messages in between each time-step to send them to the
    appropriate agents in the following step.

    IDEA:
    for t in times:
        agents_out = []
        for agent in agents:
            agent_out : list = agent.propagate(percept, t)
            agents_out.extend(agent_out)
    """
    ...

if __name__ == '__main__':
    # create agents
    
    # create wrapper

    # run wrapper

    # print results
    pass