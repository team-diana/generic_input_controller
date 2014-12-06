def clamp_center(v, below=0.2):
    """ remove values around 0 """
    if abs(v) < below:
        return 0
    else:
        return v

def clamp(l, r, v):
    if v < l:
        return l
    elif v > r:
        return r
    else:
        return v

def from_normal_to_interval(min_v, max_v, v):
    """ return a new value proportional to the interval (max_v - min_v) given value v"""
    res = (max_v - min_v)/2 * v + (min_v + max_v)/2
    return clamp(min_v, max_v, v)


