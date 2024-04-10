def constant_utility(s_max, t_img) -> float:
    return s_max

def linear_decay(s_max, t_start, t_end, t_img) -> float:
    """
    f(t<t_start) = 0.0
    f(t=t_start) = s_max
    f(t>=t_end) = 0.0
    """
    return s_max * (t_img - t_end) / (t_start - t_end) if t_start <= t_img <= t_end else 0.0 