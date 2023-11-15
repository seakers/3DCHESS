import matplotlib.pyplot as plt
import numpy as np
from subsystems import EPSubsystem
from actions import SubsystemProvidePower, SubsystemStopProvidePower
from components import Battery, Instrument, SolarPanel

if __name__ == "__main__":
    deltat = np.linspace(0,10000,10001)
    dt = 0.1

    instruments = []
    for n in range(20+np.random.randint(-5,6)):
        instruments.append(Instrument("Instrument"+str(n+2), 22+np.random.randint(-2,3), dt))
    batteries = []
    for n in range(2+np.random.randint(-1,2)):
        batteries.append(Battery("Battery"+str(n+1), 990000, dt))
    solarpanels = [SolarPanel("SolarPanel1", 300, 90, dt)]

    actions = {}
    possible_actions = [SubsystemProvidePower,SubsystemStopProvidePower]
    for n in range(20+np.random.randint(-5,16)):
        rand = np.random.randint(0,11)
        if rand >= 3: actions[np.random.randint(0,10000)] = SubsystemProvidePower(instruments[np.random.randint(0,len(instruments))].name)
        elif rand < 3: actions[np.random.randint(0,10000)] = SubsystemStopProvidePower(instruments[np.random.randint(0,len(instruments))].name)
    
    components = solarpanels + batteries + instruments

    connections = {}
    for battery in batteries:
        connection = []
        for instrument in instruments:
            connection.append([instrument, 0])
        connections[battery] = connection
    for solarpanel in solarpanels:
        connection = []
        for instrument in instruments:
            connection.append([instrument, 0])
        for battery in batteries:
            connection.append([battery, 0])
        connections[solarpanel] = connection
    EPS = EPSubsystem(components, connections,dt)

    battery1_energy = []
    for t in deltat:
        EPS.update(t)

        if t in actions:
            if EPS.perform_action(actions[t], t):
                print("action performed!")
                if isinstance(actions[t],SubsystemProvidePower):
                    plt.axvline(x=t, ymin=0, ymax=200,color='green',linestyle='--',linewidth=2)
                elif isinstance(actions[t],SubsystemStopProvidePower):
                    plt.axvline(x=t, ymin=0, ymax=200,color='red',linestyle='--',linewidth=2)
            else:
                print("action not performed!")

        battery1_energy.append(batteries[0].current_energy)
        print("t =",t,", Solar Panel load =",solarpanels[0].load)

    plt.plot(deltat, battery1_energy, label="Battery 1")
    for lapse in solarpanels[0].ECLIPSE:
        plt.axvspan(xmin=lapse[0], xmax=lapse[1], ymin=0, ymax=200, color='black', alpha=0.7)
    plt.legend()

    plt.show()