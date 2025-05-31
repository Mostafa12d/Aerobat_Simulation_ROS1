import numpy as np
from symbolic_functions_faster.func_create_wing_segmentV2 import *
# from func_create_wing_segmentV2 import *
# from func_create_wing_segment import *

class Simulation_Parameter:
    def __init__(self):
        # Setup

        # Flags during parameter setup
        self.flag_use_optimized_model = 0 # 0 = dickinson's model, 1 = load cell optimization model
        self.flag_reverse_wing = 0 # 0 = aerobat's forward sweep, 1 = reverse sweep

        # Flags during simulation
        self.flag_use_wagnermodel = 1 # 1 = wagner model, 0 = quasi-steady model

        # Tail stabilizer
        # NOTE: Tail uses quasi-steady aerodynamics
        self.flag_use_tail_stabilizer = 0 # 1 if to add a tail stabilizer

        # Simulation time parameters
        self.dt = 5e-4 # simulation time step (s). smaller is more stable but slower
        self.t_end = 10 # simulation end time (s)
        self.wait_time = 0.01 # wait time before start applying flapping speed tracking controller

        # Airflow settings
        self.airspeed = np.array([0, 0, 0]).reshape(3, 1) # freestream air speed (inertial frame)
        self.air_density = 1 # density of air (kg/m^3)

        # Flapping and crank controller gains
        self.kd = 400 # crank speed tracking controller gain
        self.flapping_freq = 4.75 # flapping frequency in Hz

        # self.flag_use_tail_stabilizer = False

        if(self.flag_use_tail_stabilizer):
            self.n_blade_tail = 2 # tail stabilizer blade elements
        else:
            self.n_blade_tail = 0

        # Blade element numbers (for lifting line theory, assume constant blade elements)
        self.n_blade_prox = 2  # number of blade elements per side (proximal)
        self.n_blade_dist = 6  # number of blade elements per side (distal)

        # Total blade elements
        self.n_blade = 2 * (self.n_blade_prox + self.n_blade_dist) + self.n_blade_tail
        self.n_Wagner = self.n_blade - self.n_blade_tail

        # KS Dimension parameters

        # NOTE: taken from solidworks

        # Linkage length parameters (meter)
        self.L1  = 7.5/1000
        self.L2a = 35/1000
        self.L2b = 5/1000 * np.cos(np.deg2rad(35))
        self.L2c = 5/1000 * np.sin(np.deg2rad(35))
        self.L3a = 50/1000
        self.L3b = 6.71/1000
        self.L3c = 6/1000
        self.L4  = 36/1000
        self.L5a = 11.18/1000
        self.L5b = 10/1000
        self.L5c = 78.78/1000
        self.L5d = -3.89/1000
        self.L6  = 36/1000
        self.L7a = 90/1000
        self.L7b = 35/1000
        self.L7c = 15/1000

        # FDC Parameters
        self.FDC_scale = np.array([[0.875862068965517, 3],  # L3b
                                   [0.875862068965517, 1.64827586206897],  # L5a
                                   [0.875862068965517, 1.16551724137931]])  # L5b
        self.FDC_range = np.array([self.L3b * self.FDC_scale[0, :],
                                   self.L5a * self.FDC_scale[1, :],
                                   self.L5b * self.FDC_scale[2, :]])
        # Initial FDC control input (nominal)
        self.FDC_ctrl = [(self.L3b - self.FDC_range[0, 0]) / (self.FDC_range[0, 1] - self.FDC_range[0, 0]),
                         (self.L5a - self.FDC_range[1, 0]) / (self.FDC_range[1, 1] - self.FDC_range[1, 0]),
                         (self.L5b - self.FDC_range[2, 0]) / (self.FDC_range[2, 1] - self.FDC_range[2, 0])]

        # Stationary joint positions (x,y) (meter)
        self.P1  = (np.array([-10, -9.8]) / 1000).reshape(2,1)
        self.P5  = (22 * np.array([np.cos(np.deg2rad(20)), np.sin(np.deg2rad(20))]) / 1000).reshape(2,1)
        self.P8  = (22 * np.array([np.sin(np.deg2rad(25)), np.cos(np.deg2rad(25))]) / 1000).reshape(2,1)

        # Wing origin offset about x-axis relative to the body COM
        self.offset_x  = np.array([-42/1000]).reshape(1,1) # wing com offset in x-axis (meter) -15 old value

        # Other parameters
        self.alpha_3 = np.deg2rad(-15) # humerus link bend angle

        # For symbolic generated function
        L = np.array([self.L1, self.L2a, self.L2b, self.L2c, self.L3a, self.L3b, self.L3c, self.alpha_3, self.L4, 
                    self.L5a, self.L5b, self.L5c, self.L5d, self.L6, self.L7a, self.L7b, self.L7c]).reshape(17,1)
        self.wing_conformation_L = np.concatenate([L, self.P1, self.P5, self.P8, self.offset_x], axis=0)
        self.wing_conformation_R = np.concatenate([L, self.P1, self.P5, self.P8, self.offset_x], axis=0)

        # Body and aerodynamic blade elements dimensions 

        # Body dimensions (meter)
        self.body_length = 40/1000  
        self.body_radius = 25/1000

        # Wing segment span (meter)
        self.span_prox = 50/1000
        self.span_dist = 132/1000

        # Proximal/inner wing segment front and rear x and y positions
        self.wing_xf_prox = np.array([63.5, 76.5]) / 1000 # front
        self.wing_xr_prox = np.array([-95.6, -111]) / 1000 # rear
        self.wing_y_prox  = np.array([0, 50]) / 1000 # y axis positions

        # Distal/outer wing segment front and rear x and y positions
        self.wing_xf_dist = np.array([76.5, 80.7, 118, 101]) / 1000 # front
        self.wing_xr_dist = np.array([-111, -115, 17.05, 101]) / 1000 # rear
        self.wing_y_dist  = np.array([0, 8.5, 84, 132]) / 1000 # y axis positions

        # Tail stabilizer dimensions (meter)
        self.tail_center = np.array([-0.12, 0, 0.01])  # tail center position
        self.tail_width = 0.050 *2 # tail width
        self.tail_chord = 0.030 *2  # tail chord
        self.tail_angle = 0.0

        # Reverse the wing sweep
        if(self.flag_reverse_wing):
        # Switch positions between wing's front and rear edge positions
            temp1 = self.wing_xf_prox
            temp2 = self.wing_xr_prox
            self.wing_xf_prox = -temp2
            self.wing_xr_prox = -temp1
            
            temp1 = self.wing_xf_dist
            temp2 = self.wing_xr_dist
            self.wing_xf_dist = -temp2
            self.wing_xr_dist = -temp1
        
        # Maximum wing span
        self.span_max = 2 * (self.body_radius + self.span_dist + self.span_prox)

        # create the wing segment data for calculating aerodynamic forces
        self.strip_id, self.strip_dc, self.strip_c, self.strip_theta = func_create_wing_segment(self)


        # Aerodynamic and mass parameters  
        
        # Wagner function coefficients, a*exp(b*t_bar)
        self.phi_a = np.array([-0.165, -0.335])   # for wagner lift model
        self.phi_b = np.array([-0.045, -0.3])
        self.Phi_0 = 1 + sum(self.phi_a)
        self.a0 = 2 * np.pi  # slope of the lift coefficient. approximately 2*pi for thin airfoil
        self.c0 = 160/1000  # base chord along wing's axis of symmetry (i'm using proximal wing's base chord)

        # Dynamic parameters
        self.chord_length = 70*10^-3

        # Aerodynamic modeling (lift and drag coefficient model)
        if(self.flag_use_optimized_model):
            # From "aero_opt_04-15-2021_exp2_v2.mat" in the simulation_data folder
            self.aero_model_lift = np.array([-0.4802, 6.5892, 2.4974, -6.3210])
            self.aero_model_drag = np.array([2.2481, -1.7594, 2.4843, -9.3407])
        else:
            # Dickinson model
            self.aero_model_lift = np.array([0.225, 1.58, 2.13, -7.2])
            self.aero_model_drag = np.array([1.92, -1.55, 2.04, -9.82])

