import numpy as np
from numba import jit

from symbolic_functions_faster.parameters_old import *
from symbolic_functions_faster.func_wing_tail import *

from utility_functions.rotation_matrix import *
import time

@jit(nopython=True,fastmath=True)
def lift_coeff(a, params):
    return params[0] + params[1] * np.sin(np.deg2rad(params[2] * np.rad2deg(a) + params[3]))

@jit(nopython=True,fastmath=True)
def drag_coeff(a, params):
    return params[0] + params[1] * np.cos(np.deg2rad(params[2] * np.rad2deg(a) + params[3]))

#parameters
def aero(xd, R_body, xa):
    #xd, R_body = original_states(model, data)

    # initialize output
    xa = xa.T
    fa = np.zeros_like(xa)  # output dxa/dt
    ua = 0  # sum of generalized aerodynamics forces
    xa_m = xa.reshape(3,nWagner).T  # reshape xa for easy access

    # --------------------------------------------------------------------------
    # calculate blade element inertial position, velocities,
    # and Ba (mapping for generalized forces)
    # --------------------------------------------------------------------------

    # Ba is the inertial force to generalized force transformation matrix for
    # calculating ua
    pos_s = np.zeros((3, n_blade))  # inertial position
    vel_s = np.zeros((3, n_blade))  # inertial velocity
    Ba_s = np.zeros((8, 3, n_blade))  # ua = Ba*fa, fa = inertial aero force
    vel_s_surf = np.zeros((3, n_blade))  # velocity about the wing axis [front, left, normal]
    e_n = np.zeros((3, n_blade))  # wing's surface normal direction
    aoa = np.zeros(n_blade)  # angle of attack (free stream)
    #aoa_d = np.zeros(n_blade)  # change in angle of attack due to downwash
    U = np.zeros(n_blade)  # effective air speed
    e_effvel = np.zeros((3, n_blade))
    start_aero = time.time()
    strip_id, strip_dc, strip_c, strip_theta = func_create_wing_segment_f()
    #print("--- Aero create wing: %s seconds ---" % (time.time() - start_aero))
    start_loop1 = time.time()
    for i in range(n_blade):
        # check which wing segment this blade element index belongs to
        if strip_id[i] == 1:  # hL
            pos, vel, Ba = func_wing_hL(xd, strip_dc[0:2, i], wing_conformation)
            Rw = rot_x(xd[0])
        elif strip_id[i] == 2:  # rL
            pos, vel, Ba = func_wing_rL(xd, strip_dc[0:2, i], wing_conformation)
            Rw = rot_x(xd[1])
        elif strip_id[i] == 3:  # hR
            pos, vel, Ba = func_wing_hR(xd, strip_dc[0:2, i], wing_conformation)
            Rw = rot_x(-xd[0])
        elif strip_id[i] == 4:  # rR
            pos, vel, Ba = func_wing_rR(xd, strip_dc[0:2, i], wing_conformation)
            Rw = rot_x(-xd[1])
        elif strip_id[i] == 5:  # body-fixed (e.g., tail)
            pos, vel, Ba = func_tail(xd, strip_dc[0:3, i])
            Rw = np.eye(3)

        # wing velocities about wing segment's axis [front, left, normal]
        # local effective velocity (wing frame)
        vel_surf = np.dot(np.transpose(Rw), np.dot(np.transpose(R_body), vel - airspeed))
        vel_surf_norm = np.linalg.norm(vel_surf)  # air velocity magnitude

        # effective velocity direction (x, z)
        if np.linalg.norm(vel_surf[[0, 2]]) < 1e-6:
            ev = np.zeros(3)  # prevent dividing by zero
        else:
            ev = vel_surf / vel_surf_norm  # unit vector of air flow direction

        alpha_inf = np.arctan2(-vel_surf[2], vel_surf[0])[0]  # angle of attack

        # record values
        pos_s[:, i] = pos.reshape(3,)
        vel_s[:, i] = np.transpose(vel - airspeed)
        vel_s_surf[:, i] = np.transpose(vel_surf)
        Ba_s[:, :, i] = Ba
        e_n[:, i] = np.dot(np.dot(R_body, Rw), np.array([0, 0, 1]))
        aoa[i] = alpha_inf
        e_effvel[:, i] = ev.flatten()  # effective velocity direction (local)

        if vel_surf_norm < 1e-4:
            U[i] = 1e-4  # prevents dividing by zero
        else:
            U[i] = vel_surf_norm
    #print("--- Aero Loop 1: %s seconds ---" % (time.time() - start_loop1))
    # -----------------------------------------------------------------------
    #  Determine An and bn for An*an_dot = bn to solve for fa
    #  Follows Boutet formulations
    #------------------------------------------------------------------------

    # Calculate An
    An = np.zeros((nWagner, nWagner))
    n = np.arange(1, nWagner+1)
    # sin_n_strip_theta = np.sin(np.outer(np.arange(1, nWagner + 1), strip_theta))
    start_loop2 = time.time()
    for i in range(nWagner):
        An[i, :] = a0 * c0 / U[i] * np.sin(n * strip_theta[i])
        # An[i, :] = a0 * c0 / U[i] * sin_n_strip_theta[i]
    #print("--- Aero Loop 2: %s seconds ---" % (time.time() - start_loop2))
    # Calculate bn
    an = xa_m[:, 0]
    bn = np.zeros((nWagner,1))
    fa_m = np.zeros((nWagner,3))
    start_loop3 = time.time()
    for i in range(nWagner):
        # aero states and effective air speed
        z1 = xa_m[i, 1]
        z2 = xa_m[i, 2]

        # downwash due to vortex
        wy = -a0 * c0 * U[i] / 4 / span_max * np.dot((n * np.sin(n * strip_theta[i])) / np.sin(strip_theta[i]),an)

        # effective downwash
        wn = vel_s_surf[2, i]
        w = wn + wy

        # alpha_downwash = np.arctan2(-w, vel_surf[0])[0] - aoa[i]

        bn[i] = -a0 * c0 / strip_c[i] * np.dot(np.sin(n * strip_theta[i]), an) + a0 * (w * Phi_0 / U[i] + phi_a[0] * phi_b[0] / (strip_c[i] / 2) * z1 + phi_a[1] * phi_b[1] * z2)

        # dz1/dt and dz2/dt
        fa_m[i, 1] = U[i] * wn + phi_b[0] / (strip_c[i] / 2) * z1 #
        fa_m[i, 2] = U[i] * wy + phi_b[1] / (strip_c[i] / 2) * z2 #
    #print("--- Aero Loop 3: %s seconds ---" % (time.time() - start_loop3))
    # calculate aerodynamic states rate of change for an
    andot = np.linalg.solve(An, bn)  # dan/dt
    fa_m[:, 0] = np.transpose(andot)
    # restructure fa_m back into vector format
    fa = np.transpose(fa_m).flatten()

    #---------------------------------------------------------------
    # Calculate aerodynamics lift and drag
    start_loop4 = time.time()
    for i in range(n_blade):
        # strip width
        if strip_id[i] == 1 or strip_id[i] == 3:
            # proximal wing
            delta_span = span_prox / n_blade_prox
        elif strip_id[i] == 2 or strip_id[i] == 4:
            # distal wing
            delta_span = span_dist / n_blade_dist
        else:
            # tail
            delta_span = tail_width / n_blade_tail

        # rotation matrix from body axis to wing axis
        if strip_id[i] == 1:  # hL
            Rw = rot_x(xd[0])
        elif strip_id[i] == 2:  # rL
            Rw = rot_x(xd[1])
        elif strip_id[i] == 3:  # hR
            Rw = rot_x(-xd[0])
        elif strip_id[i] == 4:  # rR
            Rw = rot_x(-xd[1])
        elif strip_id[i] == 5:  # tail
            Rw = np.eye(3)

        # lift coefficient (wagner for the wing, if enabled)
        if i < nWagner:
            #Gamma = 0.5 * a0 * c0 * U[i] * np.sin(strip_theta[i]) * an
            C_lift = -a0 * np.dot(np.sin(n*strip_theta[i]),(an + np.dot(strip_c[i] / U[i],fa_m[:, 0])))
        else:
            # quasi-steady model
            #Gamma = 0
            C_lift = lift_coeff(aoa[i], aero_model_lift)

        # drag coefficient (quasi-steady)
        C_drag = drag_coeff(aoa[i], aero_model_drag)

        e_lift = np.cross(e_effvel[:, i], np.array([0, 1, 0]))
        e_drag = -e_effvel[:, i]

        lift = air_density / 2 * U[i] ** 2 * C_lift * delta_span * strip_c[i] * e_lift  # Wagner
        drag = air_density / 2 * U[i] ** 2 * C_drag * delta_span * strip_c[i] * e_drag  # quasisteady

        # combine the drag and lift directions in inertial frame
        f_aero = np.dot(np.dot(R_body, Rw), (drag + lift))
        ua += np.dot(Ba_s[:, :, i], f_aero) #shape (1,8) for d2(theta5, theta6, x, y, z, roll,pitch,yaw)/dt2
    #print("--- Aero Loop 4: %s seconds ---" % (time.time() - start_loop4))
    #dynamics EOM, Md*accel + hd = ua + Jc'*lambda
    return fa,ua,xd