import numpy as np
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner


TRAJECTORY_SIZE = 33
CAMERA_OFFSET = 0.04

class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()
    self.LP = LanePlanner()
    self.readParams = 0
    self.lanelines_active = False
    self.lanelines_active_tmp = False

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.average_desired_curvature = Params().get_bool("AverageCurvature")

    self.debug_mode = debug
    self.steeringRateCost = 700.

  def update(self, sm):
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
      self.average_desired_curvature = Params().get_bool("AverageCurvature")
    elif self.readParams == 50:
      pass

    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      if self.average_desired_curvature:
        car_speed = np.array(md.velocity.x) - get_speed_error(md, v_ego_car)
      else:
        car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
      self.x_sol = np.column_stack([md.lateralPlannerSolution.x, md.lateralPlannerSolution.y, md.lateralPlannerSolution.yaw, md.lateralPlannerSolution.yawRate])

    # Parse model predictions
    self.LP.parse_model(md)
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    turn_prob = self.LP.l_turn_prob + self.LP.r_turn_prob
    # Lane change logic
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md, turn_prob, sm['navInstruction'], sm['roadLimitSpeed'], self.LP.lane_width)

    path_xyz_tmp = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz, self.lanelines_active)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.path_xyz[:,1].tolist()
    lateralPlan.psis = self.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] # TODO: unused

    lateralPlan.mpcSolutionValid = bool(1)
    lateralPlan.solverExecutionTime = 0.0
    if self.debug_mode:
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.x_sol.tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.lanelines_active
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    lateralPlan.desireEvent = self.DH.desireEvent
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.desireReady = self.DH.desireReady
    lateralPlan.apNaviDistance = int(self.DH.apNaviDistance)
    lateralPlan.apNaviSpeed = int(self.DH.apNaviSpeed)
    lateralPlan.roadEdgeStat = self.DH.prev_road_edge_stat
    lateralPlan.latDebugText = self.DH.latDebugText

    pm.send('lateralPlan', plan_send)
