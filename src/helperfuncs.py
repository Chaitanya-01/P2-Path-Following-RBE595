import numpy as np

def convrule(x,y,z,xl,yl,zl,mapscale,bloat_amount):
    return np.array([int(mapscale*(x-(xl-bloat_amount))),int(mapscale*(y-(yl-bloat_amount))),int(mapscale*(z-(zl-bloat_amount)))])
    