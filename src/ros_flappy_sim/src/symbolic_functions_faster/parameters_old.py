'''Parameters definition'''
import numpy as np
from numba import jit
flag_use_optimized_model = 0
flag_use_tail_stabilizer = 1

L1  = 7.5/1000
L2a = 35/1000
L2b = 5/1000*np.cos(np.deg2rad(35))
L2c = 5/1000*np.sin(np.deg2rad(35))
L3a = 50/1000
L3b = 6.71/1000
L3c = 6/1000
L4  = 36/1000
L5a = 11.18/1000
L5b = 10/1000
L5c = 78.78/1000
L5d = -3.89/1000
L6  = 36/1000
L7a = 90/1000
L7b = 35/1000
L7c = 15/1000

body_radius = 25/1000
span_prox   = 50/1000
span_dist   = 132/1000
span_max = 2*(body_radius + span_dist + span_prox)

#wagner function coefficients, a*exp(b*t_bar)
phi_a   = np.array([-0.165, -0.335])   # for wagner lift model
phi_b   = np.array([-0.045, -0.3])
Phi_0 = 1 + np.sum(phi_a)
a0 = 2*np.pi  # slope of the lift coefficient. approximately 2*pi for thin airfoil
c0 = 160/1000  # base chord along wing's axis of symmetry (i'm using proximal wing's base chord)


#stationary joint positions (x,y) (meter)

P1 = np.array([-10,-9.8])/1000
P5 = np.array([22*np.cos(np.deg2rad(20)), 22*np.sin(np.deg2rad(20))])/1000
P8 = np.array([22*np.sin(np.deg2rad(25)), 22*np.cos(np.deg2rad(25))])/1000
#wing origin offset about x-axis relative to the body COM
offset_x  = -42/1000 #-42/1000       # wing com offset in x-axis (meter) -15 old value

#other parameters
alpha_3 = np.deg2rad(-15)   # humerus link bend angle

wing_conformation = np.array([L1,L2a,L2b,L2c,L3a,L3b,L3c,alpha_3,L4,L5a,L5b,
                     L5c,L5d,L6,L7a,L7b,L7c,offset_x,P5[0],P5[1]])   # <========= Changes HERE

airspeed=np.array([[0]]*3)
# airspeed=np.array([[-1.65,0,0]]).T
air_density = 1


if flag_use_optimized_model == 1:
    aero_model_lift = np.array([-0.4802, 6.5892, 2.4974, -6.3210])
    aero_model_drag = np.array([2.2481, -1.7594, 2.4843, -9.3407])
else:
    # Dickinson model
    aero_model_lift = np.array([0.225, 1.58, 2.13, -7.2])
    aero_model_drag = np.array([1.92, -1.55, 2.04, -9.82])

if flag_use_tail_stabilizer:
    n_blade_tail = 2  # tail stabilizer blade elements
else:
    n_blade_tail = 0

# Blade element numbers (for lifting line theory, assume constant blade elements)
n_blade_prox = 2   # number of blade elements per side (proximal)
n_blade_dist = 6   # number of blade elements per side (distal)

# total blade elements
n_blade = 2 * (n_blade_prox + n_blade_dist) + n_blade_tail

nWagner = n_blade - n_blade_tail  # wagner model length
# Tail stabilizer dimensions in meters
tail_center = np.array([-0.12, 0, 0.01])  # Tail center position
tail_width = 0.050  # Tail width
tail_chord = 0.030  # Tail chord



# Initialize wing_xf_prox, wing_xr_prox, and wing_y_prox in meters


# Convert millimeters to meters using list comprehensions
wing_xf_prox = np.array([63.5, 76.5])/1000  # front proximity in meters
wing_xr_prox = np.array([-95.6, -111])/1000  # rear proximity in meters
wing_y_prox = np.array([0, 50])/1000  # y axis positions in meters
wing_xf_dist = np.array([76.5, 80.7, 118, 101])/1000  # front distances in meters
wing_xr_dist = np.array([-111, -115, 17.05, 101])/1000  # rear distances in meters
wing_y_dist = np.array([0, 8.5, 84, 132])/1000  # y axis positions in meters

@jit(nopython=True,fastmath=True)
def func_create_wing_segment_f():
    # blade element id, also order of segments: {1, ..., 4} = {hL, rL, hR, rR}
    idb = np.hstack((np.ones(n_blade_prox) * 1, np.ones(n_blade_dist) * 2,
                    np.ones(n_blade_prox) * 3, np.ones(n_blade_dist) * 4,
                    np.ones(n_blade_tail) * 5))

    # left wing segments positions (local frame)
    yL_prox = np.arange(1 / (2 * n_blade_prox), 1, 1/n_blade_prox) * span_prox
    xfL_prox = np.interp(yL_prox, wing_y_prox, wing_xf_prox)
    xrL_prox = np.interp(yL_prox, wing_y_prox, wing_xr_prox)

    yL_dist = np.arange(1 / (2 * n_blade_dist), 1, 1/n_blade_dist) * span_dist
    xfL_dist = np.interp(yL_dist, wing_y_dist, wing_xf_dist)
    xrL_dist = np.interp(yL_dist, wing_y_dist, wing_xr_dist)

    dc_x_prox = (xfL_prox + xrL_prox) / 2 + (xfL_prox - xrL_prox) / 4
    dc_x_dist = (xfL_dist + xrL_dist) / 2 + (xfL_dist - xrL_dist) / 4
    
    dc_x = np.hstack((dc_x_prox, dc_x_dist, dc_x_prox, dc_x_dist))
    dc_y = np.hstack((yL_prox, yL_dist, -yL_prox, -yL_dist))

    dc_wing = np.vstack((dc_x, dc_y, np.zeros_like(dc_x)))  # wing center of pressure

    # tail center of pressure positions, assume rectangular tail
    x_tail = tail_center[0] + np.ones(n_blade_tail) * tail_chord * 0.25
    y_tail = tail_center[1] + (np.arange(1 / (2 * n_blade_tail), 1, 1/n_blade_tail) - 0.5) * tail_width
    z_tail = np.ones(n_blade_tail) * tail_center[2]
    dc_tail = np.vstack((x_tail, y_tail, z_tail))

    # form dc
    dc = np.hstack((dc_wing, dc_tail))

    # Calculate element chord length
    c_prox = xfL_prox - xrL_prox
    c_dist = xfL_dist - xrL_dist
    c_wing = np.hstack((c_prox, c_dist, c_prox, c_dist))
    c_tail = np.ones(n_blade_tail) * tail_chord
    c = np.hstack((c_wing, c_tail))

    # Calculate theta
    yL = body_radius + np.hstack((yL_prox, yL_dist + span_prox))
    y = np.hstack((yL, -yL))
    theta = np.arccos(2 * y / span_max)  # not including tail, since it's quasi-steady

    return idb, dc, c, theta

