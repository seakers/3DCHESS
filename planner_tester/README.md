1- Wrapper 
    ```
    for t in times:
        agents_out = []
        for agent in agents:
            agent_out = agent.propagate(t)
            agents_out.extend(agent_out)
    ```

2- Planners

    a. CBBA Planning Phase
    
    b. CBBA Consesus Phase 