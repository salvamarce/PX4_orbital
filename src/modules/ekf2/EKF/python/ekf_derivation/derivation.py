#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: derivation.py
Description:
"""

import symforce.symbolic as sf
from derivation_utils import *

class State:
    qw = 0
    qx = 1
    qy = 2
    qz = 3
    vx = 4
    vy = 5
    vz = 6
    px = 7
    py = 8
    pz = 9
    d_ang_bx = 10
    d_ang_by = 11
    d_ang_bz = 12
    d_vel_bx = 13
    d_vel_by = 14
    d_vel_bz = 15
    ix = 16
    iy = 17
    iz = 18
    ibx = 19
    iby = 20
    ibz = 21
    wx = 22
    wy = 23
    n_states = 24

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

def predict_covariance(
        state: VState,
        P: MState,
        d_vel: sf.V3,
        d_vel_var: sf.V3,
        d_ang: sf.V3,
        d_ang_var: sf.Scalar,
        dt: sf.Scalar,
        g: sf.Scalar
):
    d_vel_b = sf.V3(state[State.d_vel_bx], state[State.d_vel_by], state[State.d_vel_bz])
    d_vel_true = d_vel - d_vel_b

    d_ang_b = sf.V3(state[State.d_ang_bx], state[State.d_ang_by], state[State.d_ang_bz])
    d_ang_true = d_ang - d_ang_b

    q = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    R_to_earth = quat_to_rot_simplified(q)
    v = sf.V3(state[State.vx], state[State.vy], state[State.vz])
    p = sf.V3(state[State.px], state[State.py], state[State.pz])

    q_new = quat_mult(q, sf.V4(1, 0.5 * d_ang_true[0],  0.5 * d_ang_true[1],  0.5 * d_ang_true[2]))
    v_new = v + R_to_earth * d_vel_true + sf.V3(0 ,0 ,g) * dt
    p_new = p + v * dt

    # predicted state vector at time t + dt
    state_new = VState.block_matrix([[q_new], [v_new], [p_new], [sf.Matrix(state[State.d_ang_bx:State.n_states])]])

    print('Computing state propagation jacobian ...')
    A = state_new.jacobian(state)
    G = state_new.jacobian(sf.V6.block_matrix([[d_vel], [d_ang]]))

    print('Computing covariance propagation ...')
    var_u = sf.Matrix.diag([d_vel_var[0], d_vel_var[1], d_vel_var[2], d_ang_var, d_ang_var, d_ang_var])
    P_new = A * P * A.T + G * var_u * G.T

    for index in range(State.n_states):
        for j in range(State.n_states):
            if index > j:
                P_new[index,j] = 0

    return P_new

def compute_airspeed_innov_and_innov_var(
        state: VState,
        P: MState,
        airspeed: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    airspeed_pred = vel_rel.norm(epsilon=epsilon)

    innov = airspeed_pred - airspeed

    H = sf.V1(airspeed_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_airspeed_h_and_k(
        state: VState,
        P: MState,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VState, VState):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    airspeed_pred = vel_rel.norm(epsilon=epsilon)
    H = sf.V1(airspeed_pred).jacobian(state)

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

def predict_sideslip(
        state: VState,
        epsilon: sf.Scalar
) -> (sf.Scalar):

    vel_rel = sf.V3(state[State.vx] - state[State.wx], state[State.vy] - state[State.wy], state[State.vz])
    q_att = sf.V4(state[State.qw], state[State.qx], state[State.qy], state[State.qz])
    relative_wind_body = quat_to_rot(q_att).T * vel_rel

    # Small angle approximation of side slip model
    # Protect division by zero using epsilon
    sideslip_pred = add_epsilon_sign(relative_wind_body[1] / relative_wind_body[0], relative_wind_body[0], epsilon)

    return sideslip_pred

def compute_sideslip_innov_and_innov_var(
        state: VState,
        P: MState,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, sf.Scalar):

    sideslip_pred = predict_sideslip(state, epsilon);

    innov = 0.0 - sideslip_pred

    H = sf.V1(sideslip_pred).jacobian(state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_sideslip_h_and_k(
        state: VState,
        P: MState,
        innov_var: sf.Scalar,
        epsilon: sf.Scalar
) -> (VState, VState):

    sideslip_pred = predict_sideslip(state, epsilon);

    H = sf.V1(sideslip_pred).jacobian(state)

    K = P * H.T / sf.Max(innov_var, epsilon)

    return (H.T, K)

generate_px4_function(compute_airspeed_innov_and_innov_var, output_names=["innov", "innov_var"])
generate_px4_function(compute_airspeed_h_and_k, output_names=["H", "K"])

generate_px4_function(compute_sideslip_innov_and_innov_var, output_names=["innov", "innov_var"])
generate_px4_function(compute_sideslip_h_and_k, output_names=["H", "K"])
generate_px4_function(predict_covariance, output_names=["P_new"])
