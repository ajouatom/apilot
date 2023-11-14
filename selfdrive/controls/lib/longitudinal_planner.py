#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.system.swaglog import cloudlog

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
#A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
#A_CRUISE_MAX_VALS = [2.0, 1.2, 0.8, 0.6]
A_CRUISE_MAX_VALS = [2.0, 1.4, 0.5, 0.2, 0.15]
A_CRUISE_MAX_BP = [0., 40 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 140 * CV.KPH_TO_MS]

# Lookup table for turns
#_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_V = [2.5, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)
    self.v_model_error = 0.0

    self.x_desired_trajectory = np.zeros(CONTROL_N)
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.param_read_counter = 0
    self.read_param()
    self.personality = log.LongitudinalPersonality.standard

    self.vCluRatio = 1.0

    self.myEcoModeFactor = 1.0
    self.cruiseMaxVals1 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
    self.cruiseMaxVals2 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    self.cruiseMaxVals3 = float(int(Params().get("CruiseMaxVals3", encoding="utf8"))) / 100.
    self.cruiseMaxVals4 = float(int(Params().get("CruiseMaxVals4", encoding="utf8"))) / 100.
    self.cruiseMaxVals5 = float(int(Params().get("CruiseMaxVals5", encoding="utf8"))) / 100.
    self.cruiseMaxVals6 = float(int(Params().get("CruiseMaxVals6", encoding="utf8"))) / 100.
    self.autoTurnControl = int(Params().get("AutoTurnControl", encoding="utf8"))

    self.mpc.openpilotLongitudinalControl = CP.openpilotLongitudinalControl

  def read_param(self):
    try:
      self.personality = int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

    self.myEcoModeFactor = float(int(Params().get("MyEcoModeFactor", encoding="utf8"))) / 100.
    self.cruiseMaxVals1 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
    self.cruiseMaxVals2 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    self.cruiseMaxVals3 = float(int(Params().get("CruiseMaxVals3", encoding="utf8"))) / 100.
    self.cruiseMaxVals4 = float(int(Params().get("CruiseMaxVals4", encoding="utf8"))) / 100.
    self.cruiseMaxVals5 = float(int(Params().get("CruiseMaxVals5", encoding="utf8"))) / 100.
    self.cruiseMaxVals6 = float(int(Params().get("CruiseMaxVals6", encoding="utf8"))) / 100.
    self.autoTurnControl = int(Params().get("AutoTurnControl", encoding="utf8"))

    
  def get_max_accel(self, v_ego):
    cruiseMaxVals = [self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3, self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6]
    return interp(v_ego, A_CRUISE_MAX_BP, cruiseMaxVals)
  @staticmethod
  def parse_model(model_msg, model_error, v_ego, autoTurnControl):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
      y = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.y)
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
      y = np.zeros(len(T_IDXS_MPC))
      
    if False: #autoTurnControl == 2: # 속도를 줄이자~
      max_lat_accel = interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)
    
    return x, v, a, j, y

  def update(self, sm):
    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1
    #self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode and self.CP.openpilotLongitudinalControl else 'acc'
    self.mpc.experimentalMode = sm['controlsState'].experimentalMode and self.CP.openpilotLongitudinalControl 

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # neokii
    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio
      #v_cruise = int(v_cruise * CV.MS_TO_KPH + 0.25) * CV.KPH_TO_MS
    mySafeModeFactor = sm['controlsState'].mySafeModeFactor
    myDrivingMode = sm['controlsState'].myDrivingMode

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]      
      if myDrivingMode in [1]: # 연비
        myMaxAccel = clip(self.get_max_accel(v_ego)*self.myEcoModeFactor, 0, ACCEL_MAX)
      elif myDrivingMode in [2]: # 안전
        myMaxAccel = clip(self.get_max_accel(v_ego)*self.myEcoModeFactor*mySafeModeFactor, 0, ACCEL_MAX)
      elif myDrivingMode in [3,4]: # 일반, 고속
        myMaxAccel = clip(self.get_max_accel(v_ego), 0, ACCEL_MAX)
      else:
        myMaxAccel = self.get_max_accel(v_ego)
      accel_limits = [A_CRUISE_MIN, myMaxAccel]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## mpc에서는 prev_a를 참고하여 constraint작동함.... pid off -> on시에는 현재 constraint가 작동하지 않아서 집어넣어봄...
      accel_limits_turns[0] = accel_limits_turns[0] = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    #self.mpc.set_weights(prev_accel_constraint)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j, y = self.parse_model(sm['modelV2'], self.v_model_error, v_ego, self.autoTurnControl)

    self.mpc.update(sm['carState'], sm['radarState'], sm['modelV2'], sm['controlsState'], v_cruise, x, v, a, j, y, prev_accel_constraint, reset_state)

    self.x_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.x_solution)
    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.x_desired_trajectory = self.x_desired_trajectory_full[:CONTROL_N]
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.distances = self.x_desired_trajectory.tolist()
    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time
    longitudinalPlan.personality = self.personality

    longitudinalPlan.debugLongText1 = self.mpc.debugLongText1
    #self.mpc.debugLongText2 = "Vout={:3.2f},{:3.2f},{:3.2f},{:3.2f},{:3.2f}".format(longitudinalPlan.speeds[0]*3.6,longitudinalPlan.speeds[1]*3.6,longitudinalPlan.speeds[2]*3.6,longitudinalPlan.speeds[3]*3.6,longitudinalPlan.speeds[-1]*3.6)
    #self.mpc.debugLongText2 = "VisionTurn:State={},Speed={:.1f}".format(self.vision_turn_controller.state, self.vision_turn_controller.v_turn*3.6)
    longitudinalPlan.debugLongText2 = self.mpc.debugLongText2
    longitudinalPlan.trafficState = self.mpc.trafficState
    longitudinalPlan.xState = self.mpc.xState
    if self.mpc.trafficError:
      longitudinalPlan.trafficState = self.mpc.trafficState + 1000
    longitudinalPlan.xStop = float(self.mpc.stopDist) #float(self.mpc.xStop)
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.cruiseGap = float(self.mpc.applyCruiseGap)
    longitudinalPlan.xObstacle = float(self.mpc.x_obstacle_min[0])
    longitudinalPlan.mpcEvent = self.mpc.mpcEvent
    longitudinalPlan.mpcMode = 1 if self.mpc.mode == 'blended' else 0

    if self.CP.openpilotLongitudinalControl:
      longitudinalPlan.xCruiseTarget = float(self.mpc.v_cruise / self.vCluRatio)
    else:
      longitudinalPlan.xCruiseTarget = float(longitudinalPlan.speeds[-1] / self.vCluRatio)

    pm.send('longitudinalPlan', plan_send)
