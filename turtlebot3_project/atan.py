import math


def rad_to_deg(rads):
    return math.degrees(rads)

def deg_to_rad(degs):
    return (math.pi/180) * degs

# origin
pos_1 = (5, 5)
t_1 = deg_to_rad(90.0)  # heading in rad (theta)

# target
pos_2 = (1.0, 1.0)
t_2 = 0.0

dx = pos_2[0] - pos_1[0]
dy = pos_2[1] - pos_1[1]


rads = math.atan2(dy, dx)  # bearing from pos_1 to pos_2 in rad
degs = rad_to_deg(rads)  # bearing from pos_1 to pos_2 in deg
print("Bearing from {0} to {1} is {2} ({3}°)".format(pos_1, pos_2, rads, degs))


heading_rad = t_1
heading_deg = rad_to_deg(heading_rad)
print("Heading of robot is {0} ({1}°)".format(heading_rad, heading_deg))

corrected_rads = rads - t_1
corrected_degs = rad_to_deg(corrected_rads)
print("Corrected Bearing from {0} to {1} is {2} ({3}°)".format(pos_1, pos_2, corrected_rads, corrected_degs))


print("0 is", rad_to_deg(0), "°")
print("0.0005 is", rad_to_deg(0.0005), "°")
print("1.5 is", rad_to_deg(1.5), "°")
print("3.14 is", rad_to_deg(3.14), "°")
print("-3.14 is", rad_to_deg(-3.14), "°")
print("-1.5 is", rad_to_deg(-1.5), "°")

print("-135 deg is", deg_to_rad(-135), "rad" )