import math

def rotate_rect(coord, theta):
    return (
        coord[0]*math.cos(theta) - coord[1]*math.sin(theta),
        coord[0]*math.sin(theta) + coord[1]*math.cos(theta),
    )
coord = rotate_rect((50, 40), math.pi/4)
print(coord)
