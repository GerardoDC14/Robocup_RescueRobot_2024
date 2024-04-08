import math

# Constants
l1 = 0.05
l2 = 0.110
l3 = 0.400
l4 = 0.390
l5 = 0.1769
l6 = 0.200
h = l1 
d = l2 

# Position values
position_x = 0.82379
position_y = 0
position_z = 0.65659

def calculate_theta_1(position_x, position_y):
    """
    Calculates theta_1 based on position_x and position_y.
    """
    return math.atan2(position_y, position_x)

def calculate_r_and_projections(position_x, position_y, position_z, d, h):
    """
    Calculates r, F, and g based on position and constants.
    """
    r = math.sqrt(position_x**2 + position_y**2)
    F = r - d
    g = position_z - h
    return r, F, g

def calculate_p_and_d(F, g, l3, l4, l5):
    """
    Calculates P and D based on F, g, and arm lengths.
    """
    P = math.sqrt(F**2 + g**2)
    D = (P**2 - l3**2 - (l4 + l5)**2) / (2 * l3 * (l4 + l5))
    return P, D

def calculate_theta_1(position_x, position_y):
    """
    Calculates theta_1 based on position_x and position_y, returns value in degrees.
    """
    return math.degrees(math.atan2(position_y, position_x))

def calculate_theta_2_and_theta_3(F, g, D, l1, l2, l3, l4, l5):
    """
    Calculates theta_2 and theta_3 based on F, g, D, and arm lengths, returns values in degrees.
    """
    theta_3 = math.degrees(math.atan2(math.sqrt(1 - D**2), D))
    alpha = math.atan2(l2 * math.sin(math.radians(theta_3)), l1 + l2 * math.cos(math.radians(theta_3)))
    gamma = math.atan2(g, F)
    theta_2 = math.degrees(gamma - alpha)
    return theta_2, theta_3

# Calculating angles in degrees
theta_1 = calculate_theta_1(position_x, position_y)
r, F, g = calculate_r_and_projections(position_x, position_y, position_z, d, h)
P, D = calculate_p_and_d(F, g, l3, l4, l5)
theta_2, theta_3 = calculate_theta_2_and_theta_3(F, g, D, l1, l2, l3, l4, l5)

# Output for verification (commented out for code finalization)
print(f"Theta 1 (degrees): {theta_1}")
print(f"Theta 2 (degrees): {theta_2}")
print(f"Theta 3 (degrees): {theta_3}")

