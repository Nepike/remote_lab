# Constant: type of agent
int8 FISH=0
int8 OBSTACLE=1
int8 BUOY=2

# agent id, should be unique among all agents
int16       id

# type of agent (could be a stationary thing, like an OBSTACLE)
int8        type

# 1.0 is the standard scale
float64     scale

# r,g,b
float64[]   color

# Cartesian x,y,z
float64[]   coordinates

# Euler angles, extrinsic (fixed axis) roll-pitch-yaw (X-Y-Z)
float64[]   rotation
