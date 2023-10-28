import numpy as np
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from openpilot.common.params import Params

TRAJECTORY_SIZE = 33
CAMERA_OFFSET = 0.04

class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()
    self.readParams = 0
    self.lanelines_active = False
    self.lanelines_active_tmp = False
    self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
    self.pathCostApply = float(int(Params().get("PathCostApply", encoding="utf8")))*0.01
    self.lateralMotionCost = float(int(Params().get("LateralMotionCost", encoding="utf8")))*0.01
    self.lateralAccelCost = float(int(Params().get("LateralAccelCost", encoding="utf8")))*0.01
    self.lateralJerkCost = float(int(Params().get("LateralJerkCost", encoding="utf8")))*0.01
    self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.l_turn_prob = 0.0
    self.r_turn_prob = 0.0

    self.debug_mode = debug
    self.steeringRateCost = 700.

  def update(self, sm):
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
      self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))
      self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
      self.pathCostApply = float(int(Params().get("PathCostApply", encoding="utf8")))*0.01
      self.steeringRateCost = float(int(Params().get("SteeringRateCost", encoding="utf8")))
    elif self.readParams == 50:
      self.lateralMotionCost = float(int(Params().get("LateralMotionCost", encoding="utf8")))*0.01
      self.lateralAccelCost = float(int(Params().get("LateralAccelCost", encoding="utf8")))*0.01
      self.lateralJerkCost = float(int(Params().get("LateralJerkCost", encoding="utf8")))*0.01

    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
      self.x_sol = np.column_stack([md.lateralPlannerSolution.x, md.lateralPlannerSolution.y, md.lateralPlannerSolution.yaw, md.lateralPlannerSolution.yawRate])

    # Lane change logic
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]
      self.l_turn_prob = desire_state[log.LateralPlan.Desire.turnLeft]
      self.r_turn_prob = desire_state[log.LateralPlan.Desire.turnRight]
    lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob
    turn_prob = self.l_turn_prob + self.r_turn_prob
    #self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md, turn_prob, sm['navInstruction'], sm['roadLimitSpeed'], 3.5)


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
    lateralPlan.laneWidth = 3.5 #float(self.LP.lane_width)
    lateralPlan.desireReady = self.DH.desireReady
    lateralPlan.apNaviDistance = int(self.DH.apNaviDistance)
    lateralPlan.apNaviSpeed = int(self.DH.apNaviSpeed)
    lateralPlan.roadEdgeStat = self.DH.prev_road_edge_stat
    lateralPlan.latDebugText = self.DH.latDebugText

    pm.send('lateralPlan', plan_send)
