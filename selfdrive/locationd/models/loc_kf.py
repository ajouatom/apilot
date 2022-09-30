#!/usr/bin/env python3

import sys

import numpy as np
import sympy as sp

from rednose.helpers.ekf_sym import EKF_sym, gen_code
from rednose.helpers.lst_sq_computer import LstSqComputer
from rednose.helpers.sympy_helpers import euler_rotate, quat_matrix_r, quat_rotate

from selfdrive.locationd.models.constants import ObservationKind
from selfdrive.locationd.models.gnss_helpers import parse_pr, parse_prr

EARTH_GM = 3.986005e14  # m^3/s^2 (gravitational constant * mass of earth)

class States():
  ECEF_POS = slice(0, 3)  # x, y and z in ECEF in meters
  ECEF_ORIENTATION = slice(3, 7)  # quat for orientation of phone in ecef
  ECEF_VELOCITY = slice(7, 10)  # ecef velocity in m/s
  ANGULAR_VELOCITY = slice(10, 13)  # roll, pitch and yaw rates in device frame in radians/s
  CLOCK_BIAS = slice(13, 14)  # clock bias in light-meters,
  CLOCK_DRIFT = slice(14, 15)  # clock drift in light-meters/s,
  GYRO_BIAS = slice(15, 18)  # roll, pitch and yaw biases
  ODO_SCALE_UNUSED = slice(18, 19)  # odometer scale
  ACCELERATION = slice(19, 22)  # Acceleration in device frame in m/s**2
  FOCAL_SCALE_UNUSED = slice(22, 23)  # focal length scale
  IMU_FROM_DEVICE_EULER = slice(23, 26)  # imu offset angles in radians
  GLONASS_BIAS = slice(26, 27)  # GLONASS bias in m expressed as bias + freq_num*freq_slope
  GLONASS_FREQ_SLOPE = slice(27, 28)  # GLONASS bias in m expressed as bias + freq_num*freq_slope
  CLOCK_ACCELERATION = slice(28, 29)  # clock acceleration in light-meters/s**2,
  ACCELEROMETER_SCALE_UNUSED = slice(29, 30)  # scale of mems accelerometer
  ACCELEROMETER_BIAS = slice(30, 33)  # bias of mems accelerometer
  # TODO the offset is likely a translation of the sensor, not a rotation of the camera
  WIDE_FROM_DEVICE_EULER = slice(33, 36)  # wide camera offset angles in radians (tici only)
  # We currently do not use ACCELEROMETER_SCALE to avoid instability due to too many free variables (ACCELEROMETER_SCALE, ACCELEROMETER_BIAS, IMU_FROM_DEVICE_EULER).
  # From experiments we see that ACCELEROMETER_BIAS is more correct than ACCELEROMETER_SCALE

  # Error-state has different slices because it is an ESKF
  ECEF_POS_ERR = slice(0, 3)
  ECEF_ORIENTATION_ERR = slice(3, 6)  # euler angles for orientation error
  ECEF_VELOCITY_ERR = slice(6, 9)
  ANGULAR_VELOCITY_ERR = slice(9, 12)
  CLOCK_BIAS_ERR = slice(12, 13)
  CLOCK_DRIFT_ERR = slice(13, 14)
  GYRO_BIAS_ERR = slice(14, 17)
  ODO_SCALE_ERR_UNUSED = slice(17, 18)
  ACCELERATION_ERR = slice(18, 21)
  FOCAL_SCALE_ERR_UNUSED = slice(21, 22)
  IMU_FROM_DEVICE_EULER_ERR = slice(22, 25)
  GLONASS_BIAS_ERR = slice(25, 26)
  GLONASS_FREQ_SLOPE_ERR = slice(26, 27)
  CLOCK_ACCELERATION_ERR = slice(27, 28)
  ACCELEROMETER_SCALE_ERR_UNUSED = slice(28, 29)
  ACCELEROMETER_BIAS_ERR = slice(29, 32)
  WIDE_FROM_DEVICE_EULER_ERR = slice(32, 35)


class LocKalman():
  name = "loc"
  x_initial = np.array([0, 0, 0,
                        1, 0, 0, 0,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0,
                        0, 0, 0,
                        1,
                        0, 0, 0,
                        1,
                        0, 0, 0,
                        0, 0,
                        0,
                        1,
                        0, 0, 0,
                        0, 0, 0], dtype=np.float64)

  # state covariance
  P_initial = np.diag([1e16, 1e16, 1e16,
                       10**2, 10**2, 10**2,
                       10**2, 10**2, 10**2,
                       1**2, 1**2, 1**2,
                       1e14, (100)**2,
                       0.05**2, 0.05**2, 0.05**2,
                       0.02**2,
                       2**2, 2**2, 2**2,
                       0.01**2,
                       0.01**2, 0.01**2, 0.01**2,
                       10**2, 1**2,
                       0.2**2,
                       0.05**2,
                       0.05**2, 0.05**2, 0.05**2,
                       0.01**2, 0.01**2, 0.01**2])


  # measurements that need to pass mahalanobis distance outlier rejector
  maha_test_kinds = [ObservationKind.ORB_FEATURES, ObservationKind.ORB_FEATURES_WIDE]  # , ObservationKind.PSEUDORANGE, ObservationKind.PSEUDORANGE_RATE]
  dim_augment = 7
  dim_augment_err = 6

  @staticmethod
  def generate_code(generated_dir, N=4):
    dim_augment = LocKalman.dim_augment
    dim_augment_err = LocKalman.dim_augment_err

    dim_main = LocKalman.x_initial.shape[0]
    dim_main_err = LocKalman.P_initial.shape[0]
    dim_state = dim_main + dim_augment * N
    dim_state_err = dim_main_err + dim_augment_err * N
    maha_test_kinds = LocKalman.maha_test_kinds

    name = f"{LocKalman.name}_{N}"

    # make functions and jacobians with sympy
    # state variables
    state_sym = sp.MatrixSymbol('state', dim_state, 1)
    state = sp.Matrix(state_sym)
    x, y, z = state[States.ECEF_POS, :]
    q = state[States.ECEF_ORIENTATION, :]
    v = state[States.ECEF_VELOCITY, :]
    vx, vy, vz = v
    omega = state[States.ANGULAR_VELOCITY, :]
    vroll, vpitch, vyaw = omega
    cb = state[States.CLOCK_BIAS, :]
    cd = state[States.CLOCK_DRIFT, :]
    roll_bias, pitch_bias, yaw_bias = state[States.GYRO_BIAS, :]
    acceleration = state[States.ACCELERATION, :]
    imu_from_device_euler = state[States.IMU_FROM_DEVICE_EULER, :]
    imu_from_device_euler[0, 0] = 0  # not observable enough
    imu_from_device_euler[2, 0] = 0  # not observable enough
    glonass_bias = state[States.GLONASS_BIAS, :]
    glonass_freq_slope = state[States.GLONASS_FREQ_SLOPE, :]
    ca = state[States.CLOCK_ACCELERATION, :]
    accel_bias = state[States.ACCELEROMETER_BIAS, :]
    wide_from_device_euler = state[States.WIDE_FROM_DEVICE_EULER, :]
    wide_from_device_euler[0, 0] = 0  # not observable enough

    dt = sp.Symbol('dt')

    # calibration and attitude rotation matrices
    quat_rot = quat_rotate(*q)

    # Got the quat predict equations from here
    # A New Quaternion-Based Kalman Filter for
    # Real-Time Attitude Estimation Using the Two-Step
    # Geometrically-Intuitive Correction Algorithm
    A = 0.5 * sp.Matrix([[0, -vroll, -vpitch, -vyaw],
                         [vroll, 0, vyaw, -vpitch],
                         [vpitch, -vyaw, 0, vroll],
                         [vyaw, vpitch, -vroll, 0]])
    q_dot = A * q

    # Time derivative of the state as a function of state
    state_dot = sp.Matrix(np.zeros((dim_state, 1)))
    state_dot[States.ECEF_POS, :] = v
    state_dot[States.ECEF_ORIENTATION, :] = q_dot
    state_dot[States.ECEF_VELOCITY, 0] = quat_rot * acceleration
    state_dot[States.CLOCK_BIAS, :] = cd
    state_dot[States.CLOCK_DRIFT, :] = ca

    # Basic descretization, 1st order intergrator
    # Can be pretty bad if dt is big
    f_sym = state + dt * state_dot

    state_err_sym = sp.MatrixSymbol('state_err', dim_state_err, 1)
    state_err = sp.Matrix(state_err_sym)
    quat_err = state_err[States.ECEF_ORIENTATION_ERR, :]
    v_err = state_err[States.ECEF_VELOCITY_ERR, :]
    omega_err = state_err[States.ANGULAR_VELOCITY_ERR, :]
    cd_err = state_err[States.CLOCK_DRIFT_ERR, :]
    acceleration_err = state_err[States.ACCELERATION_ERR, :]
    ca_err = state_err[States.CLOCK_ACCELERATION_ERR, :]

    # Time derivative of the state error as a function of state error and state
    quat_err_matrix = euler_rotate(quat_err[0], quat_err[1], quat_err[2])
    q_err_dot = quat_err_matrix * quat_rot * (omega + omega_err)
    state_err_dot = sp.Matrix(np.zeros((dim_state_err, 1)))
    state_err_dot[States.ECEF_POS_ERR, :] = v_err
    state_err_dot[States.ECEF_ORIENTATION_ERR, :] = q_err_dot
    state_err_dot[States.ECEF_VELOCITY_ERR, :] = quat_err_matrix * quat_rot * (acceleration + acceleration_err)
    state_err_dot[States.CLOCK_BIAS_ERR, :] = cd_err
    state_err_dot[States.CLOCK_DRIFT_ERR, :] = ca_err
    f_err_sym = state_err + dt * state_err_dot

    # convenient indexing
    # q idxs are for quats and p idxs are for other
    q_idxs = [[3, dim_augment]] + [[dim_main + n * dim_augment + 3, dim_main + (n + 1) * dim_augment] for n in range(N)]
    q_err_idxs = [[3, dim_augment_err]] + [[dim_main_err + n * dim_augment_err + 3, dim_main_err + (n + 1) * dim_augment_err] for n in range(N)]
    p_idxs = [[0, 3]] + [[dim_augment, dim_main]] + [[dim_main + n * dim_augment, dim_main + n * dim_augment + 3] for n in range(N)]
    p_err_idxs = [[0, 3]] + [[dim_augment_err, dim_main_err]] + [[dim_main_err + n * dim_augment_err, dim_main_err + n * dim_augment_err + 3] for n in range(N)]

    # Observation matrix modifier
    H_mod_sym = sp.Matrix(np.zeros((dim_state, dim_state_err)))
    for p_idx, p_err_idx in zip(p_idxs, p_err_idxs):
      H_mod_sym[p_idx[0]:p_idx[1], p_err_idx[0]:p_err_idx[1]] = np.eye(p_idx[1] - p_idx[0])
    for q_idx, q_err_idx in zip(q_idxs, q_err_idxs):
      H_mod_sym[q_idx[0]:q_idx[1], q_err_idx[0]:q_err_idx[1]] = 0.5 * quat_matrix_r(state[q_idx[0]:q_idx[1]])[:, 1:]

    # these error functions are defined so that say there
    # is a nominal x and true x:
    # true x = err_function(nominal x, delta x)
    # delta x = inv_err_function(nominal x, true x)
    nom_x = sp.MatrixSymbol('nom_x', dim_state, 1)
    true_x = sp.MatrixSymbol('true_x', dim_state, 1)
    delta_x = sp.MatrixSymbol('delta_x', dim_state_err, 1)

    err_function_sym = sp.Matrix(np.zeros((dim_state, 1)))
    for q_idx, q_err_idx in zip(q_idxs, q_err_idxs):
      delta_quat = sp.Matrix(np.ones(4))
      delta_quat[1:, :] = sp.Matrix(0.5 * delta_x[q_err_idx[0]: q_err_idx[1], :])
      err_function_sym[q_idx[0]:q_idx[1], 0] = quat_matrix_r(nom_x[q_idx[0]:q_idx[1], 0]) * delta_quat
    for p_idx, p_err_idx in zip(p_idxs, p_err_idxs):
      err_function_sym[p_idx[0]:p_idx[1], :] = sp.Matrix(nom_x[p_idx[0]:p_idx[1], :] + delta_x[p_err_idx[0]:p_err_idx[1], :])

    inv_err_function_sym = sp.Matrix(np.zeros((dim_state_err, 1)))
    for p_idx, p_err_idx in zip(p_idxs, p_err_idxs):
      inv_err_function_sym[p_err_idx[0]:p_err_idx[1], 0] = sp.Matrix(-nom_x[p_idx[0]:p_idx[1], 0] + true_x[p_idx[0]:p_idx[1], 0])
    for q_idx, q_err_idx in zip(q_idxs, q_err_idxs):
      delta_quat = quat_matrix_r(nom_x[q_idx[0]:q_idx[1], 0]).T * true_x[q_idx[0]:q_idx[1], 0]
      inv_err_function_sym[q_err_idx[0]:q_err_idx[1], 0] = sp.Matrix(2 * delta_quat[1:])

    eskf_params = [[err_function_sym, nom_x, delta_x],
                   [inv_err_function_sym, nom_x, true_x],
                   H_mod_sym, f_err_sym, state_err_sym]
    #
    # Observation functions
    #

    # extra args
    sat_pos_freq_sym = sp.MatrixSymbol('sat_pos', 4, 1)
    sat_pos_vel_sym = sp.MatrixSymbol('sat_pos_vel', 6, 1)
    # sat_los_sym = sp.MatrixSymbol('sat_los', 3, 1)

    # expand extra args
    sat_x, sat_y, sat_z, glonass_freq = sat_pos_freq_sym
    sat_vx, sat_vy, sat_vz = sat_pos_vel_sym[3:]

    h_pseudorange_sym = sp.Matrix([
      sp.sqrt(
        (x - sat_x)**2 +
        (y - sat_y)**2 +
        (z - sat_z)**2
      ) + cb[0]
    ])

    h_pseudorange_glonass_sym = sp.Matrix([
      sp.sqrt(
        (x - sat_x)**2 +
        (y - sat_y)**2 +
        (z - sat_z)**2
      ) + cb[0] + glonass_bias[0] + glonass_freq_slope[0] * glonass_freq
    ])

    los_vector = (sp.Matrix(sat_pos_vel_sym[0:3]) - sp.Matrix([x, y, z]))
    los_vector = los_vector / sp.sqrt(los_vector[0]**2 + los_vector[1]**2 + los_vector[2]**2)
    h_pseudorange_rate_sym = sp.Matrix([los_vector[0] * (sat_vx - vx) +
                                        los_vector[1] * (sat_vy - vy) +
                                        los_vector[2] * (sat_vz - vz) +
                                        cd[0]])

    imu_from_device = euler_rotate(*imu_from_device_euler)
    h_gyro_sym = imu_from_device * sp.Matrix([vroll + roll_bias,
                                      vpitch + pitch_bias,
                                      vyaw + yaw_bias])

    pos = sp.Matrix([x, y, z])
    # add 1 for stability, prevent division by 0
    gravity = quat_rot.T * ((EARTH_GM / ((x**2 + y**2 + z**2 + 1)**(3.0 / 2.0))) * pos)
    h_acc_sym = imu_from_device * (gravity + acceleration + accel_bias)
    h_acc_stationary_sym = acceleration
    h_phone_rot_sym = sp.Matrix([vroll, vpitch, vyaw])
    h_relative_motion = sp.Matrix(quat_rot.T * v)

    obs_eqs = [[h_gyro_sym, ObservationKind.PHONE_GYRO, None],
               [h_phone_rot_sym, ObservationKind.NO_ROT, None],
               [h_acc_sym, ObservationKind.PHONE_ACCEL, None],
               [h_pseudorange_sym, ObservationKind.PSEUDORANGE_GPS, sat_pos_freq_sym],
               [h_pseudorange_glonass_sym, ObservationKind.PSEUDORANGE_GLONASS, sat_pos_freq_sym],
               [h_pseudorange_rate_sym, ObservationKind.PSEUDORANGE_RATE_GPS, sat_pos_vel_sym],
               [h_pseudorange_rate_sym, ObservationKind.PSEUDORANGE_RATE_GLONASS, sat_pos_vel_sym],
               [h_relative_motion, ObservationKind.CAMERA_ODO_TRANSLATION, None],
               [h_phone_rot_sym, ObservationKind.CAMERA_ODO_ROTATION, None],
               [h_acc_stationary_sym, ObservationKind.NO_ACCEL, None]]

    wide_from_device = euler_rotate(*wide_from_device_euler)
    # MSCKF configuration
    if N > 0:
      # experimentally found this is correct value for imx298 with 910 focal length
      # this is a variable so it can change with focus, but we disregard that for now
      # TODO: this isn't correct for tici
      focal_scale = 1.01
      # Add observation functions for orb feature tracks
      track_epos_sym = sp.MatrixSymbol('track_epos_sym', 3, 1)
      track_x, track_y, track_z = track_epos_sym
      h_track_sym = sp.Matrix(np.zeros(((1 + N) * 2, 1)))
      h_track_wide_cam_sym = sp.Matrix(np.zeros(((1 + N) * 2, 1)))

      track_pos_sym = sp.Matrix([track_x - x, track_y - y, track_z - z])
      track_pos_rot_sym = quat_rot.T * track_pos_sym
      track_pos_rot_wide_cam_sym = wide_from_device * track_pos_rot_sym
      h_track_sym[-2:, :] = sp.Matrix([focal_scale * (track_pos_rot_sym[1] / track_pos_rot_sym[0]),
                                       focal_scale * (track_pos_rot_sym[2] / track_pos_rot_sym[0])])
      h_track_wide_cam_sym[-2:, :] = sp.Matrix([focal_scale * (track_pos_rot_wide_cam_sym[1] / track_pos_rot_wide_cam_sym[0]),
                                                focal_scale * (track_pos_rot_wide_cam_sym[2] / track_pos_rot_wide_cam_sym[0])])

      h_msckf_test_sym = sp.Matrix(np.zeros(((1 + N) * 3, 1)))
      h_msckf_test_sym[-3:, :] = track_pos_sym

      for n in range(N):
        idx = dim_main + n * dim_augment
        # err_idx = dim_main_err + n * dim_augment_err  # FIXME: Why is this not used?
        x, y, z = state[idx:idx + 3]
        q = state[idx + 3:idx + 7]
        quat_rot = quat_rotate(*q)
        track_pos_sym = sp.Matrix([track_x - x, track_y - y, track_z - z])
        track_pos_rot_sym = quat_rot.T * track_pos_sym
        track_pos_rot_wide_cam_sym = wide_from_device * track_pos_rot_sym
        h_track_sym[n * 2:n * 2 + 2, :] = sp.Matrix([focal_scale * (track_pos_rot_sym[1] / track_pos_rot_sym[0]),
                                                     focal_scale * (track_pos_rot_sym[2] / track_pos_rot_sym[0])])
        h_track_wide_cam_sym[n * 2: n * 2 + 2, :] = sp.Matrix([focal_scale * (track_pos_rot_wide_cam_sym[1] / track_pos_rot_wide_cam_sym[0]),
                                                               focal_scale * (track_pos_rot_wide_cam_sym[2] / track_pos_rot_wide_cam_sym[0])])
        h_msckf_test_sym[n * 3:n * 3 + 3, :] = track_pos_sym

      obs_eqs.append([h_msckf_test_sym, ObservationKind.MSCKF_TEST, track_epos_sym])
      obs_eqs.append([h_track_sym, ObservationKind.ORB_FEATURES, track_epos_sym])
      obs_eqs.append([h_track_wide_cam_sym, ObservationKind.ORB_FEATURES_WIDE, track_epos_sym])
      obs_eqs.append([h_track_sym, ObservationKind.FEATURE_TRACK_TEST, track_epos_sym])
      msckf_params = [dim_main, dim_augment, dim_main_err, dim_augment_err, N, [ObservationKind.MSCKF_TEST, ObservationKind.ORB_FEATURES, ObservationKind.ORB_FEATURES_WIDE]]
    else:
      msckf_params = None
    gen_code(generated_dir, name, f_sym, dt, state_sym, obs_eqs, dim_state, dim_state_err, eskf_params, msckf_params, maha_test_kinds)

  def __init__(self, generated_dir, N=4, erratic_clock=False):
    name = f"{self.name}_{N}"


    # process noise
    clock_error_drift = 100.0 if erratic_clock else 0.1
    self.Q = np.diag([0.03**2, 0.03**2, 0.03**2,
                      0.0**2, 0.0**2, 0.0**2,
                      0.0**2, 0.0**2, 0.0**2,
                      0.1**2, 0.1**2, 0.1**2,
                      (clock_error_drift)**2, (0)**2,
                      (0.005 / 100)**2, (0.005 / 100)**2, (0.005 / 100)**2,
                      (0.02 / 100)**2,
                      3**2, 3**2, 3**2,
                      0.001**2,
                      (0.05 / 60)**2, (0.05 / 60)**2, (0.05 / 60)**2,
                      (.1)**2, (.01)**2,
                      0.005**2,
                      (0.02 / 100)**2,
                      (0.005 / 100)**2, (0.005 / 100)**2, (0.005 / 100)**2,
                      (0.05 / 60)**2, (0.05 / 60)**2, (0.05 / 60)**2])


    self.obs_noise = {ObservationKind.ODOMETRIC_SPEED: np.atleast_2d(0.2**2),
                      ObservationKind.PHONE_GYRO: np.diag([0.025**2, 0.025**2, 0.025**2]),
                      ObservationKind.PHONE_ACCEL: np.diag([.5**2, .5**2, .5**2]),
                      ObservationKind.CAMERA_ODO_ROTATION: np.diag([0.05**2, 0.05**2, 0.05**2]),
                      ObservationKind.IMU_FRAME: np.diag([0.05**2, 0.05**2, 0.05**2]),
                      ObservationKind.NO_ROT: np.diag([0.0025**2, 0.0025**2, 0.0025**2]),
                      ObservationKind.ECEF_POS: np.diag([5**2, 5**2, 5**2]),
                      ObservationKind.NO_ACCEL: np.diag([0.0025**2, 0.0025**2, 0.0025**2])}

    # MSCKF stuff
    self.N = N
    self.dim_main = LocKalman.x_initial.shape[0]
    self.dim_main_err = LocKalman.P_initial.shape[0]
    self.dim_state = self.dim_main + self.dim_augment * self.N
    self.dim_state_err = self.dim_main_err + self.dim_augment_err * self.N

    if self.N > 0:
      x_initial, P_initial, Q = self.pad_augmented(self.x_initial, self.P_initial, self.Q)  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
      self.computer = LstSqComputer(generated_dir, N)

    self.quaternion_idxs = [3, ] + [(self.dim_main + i * self.dim_augment + 3)for i in range(self.N)]

    # init filter
    self.filter = EKF_sym(generated_dir, name, Q, x_initial, P_initial, self.dim_main, self.dim_main_err,
                          N, self.dim_augment, self.dim_augment_err, self.maha_test_kinds, self.quaternion_idxs)

  @property
  def x(self):
    return self.filter.state()

  @property
  def t(self):
    return self.filter.get_filter_time()

  @property
  def P(self):
    return self.filter.covs()

  def predict(self, t):
    return self.filter.predict(t)

  def rts_smooth(self, estimates):
    return self.filter.rts_smooth(estimates, norm_quats=True)

  def pad_augmented(self, x, P, Q=None):
    if x.shape[0] == self.dim_main and self.N > 0:
      x = np.pad(x, (0, self.N * self.dim_augment), mode='constant')
      x[self.dim_main + 3::7] = 1
    if P.shape[0] == self.dim_main_err and self.N > 0:
      P = np.pad(P, [(0, self.N * self.dim_augment_err), (0, self.N * self.dim_augment_err)], mode='constant')
      P[self.dim_main_err:, self.dim_main_err:] = 10e20 * np.eye(self.dim_augment_err * self.N)
    if Q is None:
      return x, P
    else:
      Q = np.pad(Q, [(0, self.N * self.dim_augment_err), (0, self.N * self.dim_augment_err)], mode='constant')
      return x, P, Q

  def init_state(self, state, covs_diag=None, covs=None, filter_time=None):
    if covs_diag is not None:
      P = np.diag(covs_diag)
    elif covs is not None:
      P = covs
    else:
      P = self.filter.covs()
    state, P = self.pad_augmented(state, P)
    self.filter.init_state(state, P, filter_time)

  def predict_and_observe(self, t, kind, data):
    if len(data) > 0:
      data = np.atleast_2d(data)
    if kind == ObservationKind.CAMERA_ODO_TRANSLATION:
      r = self.predict_and_update_odo_trans(data, t, kind)
    elif kind == ObservationKind.CAMERA_ODO_ROTATION:
      r = self.predict_and_update_odo_rot(data, t, kind)
    elif kind == ObservationKind.PSEUDORANGE_GPS or kind == ObservationKind.PSEUDORANGE_GLONASS:
      r = self.predict_and_update_pseudorange(data, t, kind)
    elif kind == ObservationKind.PSEUDORANGE_RATE_GPS or kind == ObservationKind.PSEUDORANGE_RATE_GLONASS:
      r = self.predict_and_update_pseudorange_rate(data, t, kind)
    elif kind == ObservationKind.ORB_FEATURES or kind == ObservationKind.ORB_FEATURES_WIDE:
      r = self.predict_and_update_orb_features(data, t, kind)
    elif kind == ObservationKind.MSCKF_TEST:
      r = self.predict_and_update_msckf_test(data, t, kind)
    else:
      r = self.filter.predict_and_update_batch(t, kind, data, self.get_R(kind, len(data)))
    # Normalize quats
    quat_norm = np.linalg.norm(self.filter.state()[3:7])
    # Should not continue if the quats behave this weirdly
    if not 0.1 < quat_norm < 10:
      raise RuntimeError("Sir! The filter's gone all wobbly!")
    return r

  def get_R(self, kind, n):
    obs_noise = self.obs_noise[kind]
    dim = obs_noise.shape[0]
    R = np.zeros((n, dim, dim))
    for i in range(n):
      R[i, :, :] = obs_noise
    return R

  def predict_and_update_pseudorange(self, meas, t, kind):
    R = np.zeros((len(meas), 1, 1))
    sat_pos_freq = np.zeros((len(meas), 4))
    z = np.zeros((len(meas), 1))
    for i, m in enumerate(meas):
      z_i, R_i, sat_pos_freq_i = parse_pr(m)
      sat_pos_freq[i, :] = sat_pos_freq_i
      z[i, :] = z_i
      R[i, :, :] = R_i
    return self.filter.predict_and_update_batch(t, kind, z, R, sat_pos_freq)

  def predict_and_update_pseudorange_rate(self, meas, t, kind):
    R = np.zeros((len(meas), 1, 1))
    z = np.zeros((len(meas), 1))
    sat_pos_vel = np.zeros((len(meas), 6))
    for i, m in enumerate(meas):
      z_i, R_i, sat_pos_vel_i = parse_prr(m)
      sat_pos_vel[i] = sat_pos_vel_i
      R[i, :, :] = R_i
      z[i, :] = z_i
    return self.filter.predict_and_update_batch(t, kind, z, R, sat_pos_vel)

  def predict_and_update_odo_trans(self, trans, t, kind):
    z = trans[:, :3]
    R = np.zeros((len(trans), 3, 3))
    for i, _ in enumerate(z):
      R[i, :, :] = np.diag(trans[i, 3:]**2)
    return self.filter.predict_and_update_batch(t, kind, z, R)

  def predict_and_update_odo_rot(self, rot, t, kind):
    z = rot[:, :3]
    R = np.zeros((len(rot), 3, 3))
    for i, _ in enumerate(z):
      R[i, :, :] = np.diag(rot[i, 3:]**2)
    return self.filter.predict_and_update_batch(t, kind, z, R)

  def predict_and_update_orb_features(self, tracks, t, kind):
    k = 2 * (self.N + 1)
    R = np.zeros((len(tracks), k, k))
    z = np.zeros((len(tracks), k))
    ecef_pos = np.zeros((len(tracks), 3))
    ecef_pos[:] = np.nan
    poses = self.x[self.dim_main:].reshape((-1, 7))
    times = tracks.reshape((len(tracks), self.N + 1, 4))[:, :, 0]
    if kind==ObservationKind.ORB_FEATURES:
      pt_std = 0.005
    else:
      pt_std = 0.02
    if times.any():
      assert np.allclose(times[0, :-1], self.filter.get_augment_times(), atol=1e-7, rtol=0.0)
      for i, track in enumerate(tracks):
        img_positions = track.reshape((self.N + 1, 4))[:, 2:]

        # TODO not perfect as last pose not used
        # img_positions = unroll_shutter(img_positions, poses, self.filter.state()[7:10], self.filter.state()[10:13], ecef_pos[i])

        ecef_pos[i] = self.computer.compute_pos(poses, img_positions[:-1])
        z[i] = img_positions.flatten()
        R[i, :, :] = np.diag([pt_std**2] * (k))

    good_idxs = np.all(np.isfinite(ecef_pos), axis=1)

    # This code relies on wide and narrow orb features being captured at the same time,
    # and wide features to be processed first.
    ret = self.filter.predict_and_update_batch(t, kind, z[good_idxs], R[good_idxs], ecef_pos[good_idxs],
                                               augment=kind==ObservationKind.ORB_FEATURES)
    if ret is None:
      return

    # have to do some weird stuff here to keep
    # to have the observations input from mesh3d
    # consistent with the outputs of the filter
    # Probably should be replaced, not sure how.
    y_full = np.zeros((z.shape[0], z.shape[1] - 3))
    if sum(good_idxs) > 0:
      y_full[good_idxs] = np.array(ret[6])
    ret = ret[:6] + (y_full, z, ecef_pos)
    return ret

  def predict_and_update_msckf_test(self, test_data, t, kind):
    assert self.N > 0
    z = test_data
    R = np.zeros((len(test_data), len(z[0]), len(z[0])))
    ecef_pos = [self.x[:3]]
    for i, _ in enumerate(z):
      R[i, :, :] = np.diag([0.1**2] * len(z[0]))
    ret = self.filter.predict_and_update_batch(t, kind, z, R, ecef_pos)
    self.filter.augment()
    return ret

  def maha_test_pseudorange(self, x, P, meas, kind, maha_thresh=.3):
    bools = []
    for m in meas:
      z, R, sat_pos_freq = parse_pr(m)
      bools.append(self.filter.maha_test(x, P, kind, z, R, extra_args=sat_pos_freq, maha_thresh=maha_thresh))
    return np.array(bools)

  def maha_test_pseudorange_rate(self, x, P, meas, kind, maha_thresh=.999):
    bools = []
    for m in meas:
      z, R, sat_pos_vel = parse_prr(m)
      bools.append(self.filter.maha_test(x, P, kind, z, R, extra_args=sat_pos_vel, maha_thresh=maha_thresh))
    return np.array(bools)


if __name__ == "__main__":
  N = int(sys.argv[1].split("_")[-1])
  generated_dir = sys.argv[2]
  LocKalman.generate_code(generated_dir, N=N)
