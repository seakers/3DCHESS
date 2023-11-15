from abc import ABC

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
    # ComponentProvide Power
    
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
    # ComponentProvide Power
    
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
    # ComponentChargeBattery
    
    The action made by the EPS Subsystem to tell a solar panel to charge a battery
    """
    def __init__(   self,
                    charging_power : float,
                    t_start : float = 0.0,
                    status : str = ComponentAction.PENDING
                    ) -> None:

        super().__init__(t_start, status)

        self.charging_power = charging_power