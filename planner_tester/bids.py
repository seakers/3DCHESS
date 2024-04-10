class Bid:
    def __init__(self, 
                 task : dict,
                 score : float, 
                 winner : str, 
                 t_update : float, 
                 t_img : float) -> None:
        
        self.task = task
        self.score = score
        self.winner = winner
        self.t_update = t_update
        self.t_img = t_img