#!/usr/bin/env python3
import os
import numpy as np

from common.realtime import sec_since_boot
from common.numpy_fast import clip, interp
from selfdrive.swaglog import cloudlog
from selfdrive.modeld.constants import index_function
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from common.conversions import Conversions as CV
from common.params import Params
from common.realtime import DT_MDL
from common.filter_simple import StreamingMovingAverage
from cereal import log

XState = log.LongitudinalPlan.XState

if __name__ == '__main__':  # generating code
  from third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 6.#3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


DIFF_RADAR_VISION = 2.0
# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
MIN_ACCEL = -4.0 #-3.5
MAX_ACCEL = 2.5
T_FOLLOW = 1.45
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.5

def get_stopped_equivalence_factor(v_lead, v_ego, t_follow=T_FOLLOW, stop_distance=STOP_DISTANCE, krkeegan=False):
  if not krkeegan:
    return (v_lead**2) / (2 * COMFORT_BRAKE)

  # KRKeegan방법은 lead거리값을 고의로 늘려주어. solver가 더빠른 가속을 유발하도록 하는것 같음...
  # KRKeegan this offset rapidly decreases the following distance when the lead pulls
  # away, resulting in an early demand for acceleration.
  v_diff_offset = 0
  if np.all(v_lead - v_ego > 0):
    v_diff_offset = ((v_lead - v_ego) * 1.)
    v_diff_offset = np.clip(v_diff_offset, 0, stop_distance / 2)
    v_diff_offset = np.maximum(v_diff_offset * ((10 - v_ego)/10), 0)
  distance = (v_lead**2) / (2 * COMFORT_BRAKE) + v_diff_offset
  return distance

def get_safe_obstacle_distance(v_ego, t_follow=T_FOLLOW, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead):
  return get_safe_obstacle_distance(v_ego) - get_stopped_equivalence_factor(v_lead, v_ego)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, comfort_brake, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, T_FOLLOW, LEAD_DANGER_FACTOR, COMFORT_BRAKE, STOP_DISTANCE])

  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc'):
    self.mode = mode
    self.debugLongText1 = ""
    self.debugLongText2 = ""
    self.trafficState = 0
    self.XEgoObstacleCost = 3.
    self.JEgoCost = 5.
    self.AChangeCost = 200.
    self.DangerZoneCost = 100.
    self.leadDangerFactor = LEAD_DANGER_FACTOR
    self.trafficStopDistanceAdjust = 0.
    self.applyLongDynamicCost = False
    self.trafficStopAccel = 1.
    self.trafficStopModelSpeed = True
    self.e2eDecelSpeed = 0
    self.applyDynamicTFollow = 1.0
    self.applyDynamicTFollowApart = 1.0
    self.applyDynamicTFollowDecel = 1.0
    self.softHoldMode = 1
    self.tFollowRatio = 1.0
    self.stopDistance = STOP_DISTANCE
    self.softHoldTimer = 0
    self.lo_timer = 0 
    self.v_cruise = 0.
    self.xStopFilter = StreamingMovingAverage(3)  #11
    self.xStopFilter2 = StreamingMovingAverage(15) #3
    self.vFilter = StreamingMovingAverage(3)
    self.buttonStopDist = 0
    self.applyCruiseGap = 1.
    self.applyModelDistOrder = 32

    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.xStop = 0.0
    self.e2ePaused = False
    self.trafficError = False
    self.longActiveUser = 0
    self.cruiseButtonCounter = 0
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]
    self.x_obstacle_min = 0.0
    self.openpilotLongitudinalControl = False

  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.startSignCount = 0
    self.stopSignCount = 0
    self.longActiveUser_prev = 0

    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def get_cost_multipliers(self, v_lead0, v_lead1):
    v_ego = self.x0[1]
    v_ego_bps = [0, 10]
    TFs = [1.2, 1.45, 1.8]
    # KRKeegan adjustments to costs for different TFs
    # these were calculated using the test_longitudial.py deceleration tests
  #TF에 의한 a,j,d cost변경
    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) # 가까울수록 작게
    j_ego_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) #가까울수록 작게
    d_zone_tf = interp(self.t_follow, TFs, [1.3, 1., 1.]) # 가까울수록 크게

    # KRKeegan adjustments to improve sluggish acceleration
    # do not apply to deceleration
    j_ego_v_ego = 1
    a_change_v_ego = 1
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):  #상대차량이 현재속도보다 빠르다면...
      j_ego_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])
      a_change_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])
    # Select the appropriate min/max of the options
    j_ego = min(j_ego_tf, j_ego_v_ego)
    a_change = min(a_change_tf, a_change_v_ego)
    return (a_change, j_ego, d_zone_tf)

  def set_weights(self, prev_accel_constraint=True, v_lead0=0, v_lead1=0):
    if self.mode == 'acc':
      a_change_cost = self.AChangeCost if prev_accel_constraint else 0
      if self.applyLongDynamicCost:
        cost_mulitpliers = self.get_cost_multipliers(v_lead0, v_lead1)
        cost_weights = [self.XEgoObstacleCost, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost * cost_mulitpliers[0], self.JEgoCost * cost_mulitpliers[1]]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, self.DangerZoneCost * cost_mulitpliers[2]]
      else:
        cost_weights = [self.XEgoObstacleCost, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost, self.JEgoCost]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, self.DangerZoneCost]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(0, N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      #x_lead = lead.dRel if lead.radar else max(lead.dRel-DIFF_RADAR_VISION, 0.)
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-MIN_ACCEL * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, carstate, radarstate, model, controls, v_cruise, x, v, a, j, y, prev_accel_constraint):
    v_ego = self.x0[1]
    a_ego = carstate.aEgo

    model_x = model.position.x[-1]
    model_stop_x = model.position.x[self.applyModelDistOrder]
    self.lo_timer += 1
    if self.lo_timer > 200:
      self.lo_timer = 0
      self.XEgoObstacleCost = float(int(Params().get("XEgoObstacleCost", encoding="utf8")))
      self.JEgoCost = float(int(Params().get("JEgoCost", encoding="utf8")))
    elif self.lo_timer == 20:
      self.AChangeCost = float(int(Params().get("AChangeCost", encoding="utf8")))
      self.DangerZoneCost = float(int(Params().get("DangerZoneCost", encoding="utf8")))
    elif self.lo_timer == 40:
      self.leadDangerFactor = float(int(Params().get("LeadDangerFactor", encoding="utf8"))) * 0.01
      self.trafficStopDistanceAdjust = float(int(Params().get("TrafficStopDistanceAdjust", encoding="utf8"))) / 100.
    elif self.lo_timer == 60:
      self.applyLongDynamicCost = Params().get_bool("ApplyLongDynamicCost")
      self.trafficStopAccel = float(int(Params().get("TrafficStopAccel", encoding="utf8"))) / 100.
    elif self.lo_timer == 80:
      self.trafficStopModelSpeed = Params().get_bool("TrafficStopModelSpeed")
      self.stopDistance = float(int(Params().get("StopDistance", encoding="utf8"))) / 100.
    elif self.lo_timer == 100:
      self.e2eDecelSpeed = float(int(Params().get("E2eDecelSpeed", encoding="utf8")))
      self.applyDynamicTFollow = float(int(Params().get("ApplyDynamicTFollow", encoding="utf8"))) / 100.
    elif self.lo_timer == 120:
      self.applyDynamicTFollowApart = float(int(Params().get("ApplyDynamicTFollowApart", encoding="utf8"))) / 100.
      self.applyDynamicTFollowDecel = float(int(Params().get("ApplyDynamicTFollowDecel", encoding="utf8"))) / 100.
    elif self.lo_timer == 140:
      self.tFollowRatio = float(int(Params().get("TFollowRatio", encoding="utf8"))) / 100.     
      self.softHoldMode = int(Params().get("SoftHoldMode", encoding="utf8"))
    elif self.lo_timer == 160:
      self.applyModelDistOrder = int(Params().get("ApplyModelDistOrder", encoding="utf8"))

    #self.trafficState = 0
    self.debugLongText1 = ""
    mySafeModeFactor = clip(controls.mySafeModeFactor, 0.5, 1.0)

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    v_ego_kph = v_ego * CV.MS_TO_KPH

    if controls.longCruiseGap >= 4:
      self.applyCruiseGap = interp(v_ego_kph, [0, 45, 60, 100, 120, 140], [1,1,2,2,3,4])
      cruiseGapRatio = interp(self.applyCruiseGap, [1,2,3,4], [1.1, 1.2, 1.3, 1.45])
      self.applyCruiseGap = clip(self.applyCruiseGap, 1, 4)
    else:
      self.applyCruiseGap = float(controls.longCruiseGap)
      cruiseGapRatio = interp(controls.longCruiseGap, [1,2,3], [1.1, 1.3, 1.6])

    self.t_follow = max(0.9, cruiseGapRatio * self.tFollowRatio * (2.0 - mySafeModeFactor)) # 0.9아래는 위험하니 적용안함.


    # lead값을 고의로 줄여주면, 빨리 감속, lead값을 늘려주면 빨리가속,
    if radarstate.leadOne.status:
      self.t_follow *= interp(radarstate.leadOne.vRel*3.6, [-100., 0, 100.], [self.applyDynamicTFollow, 1.0, self.applyDynamicTFollowApart])
      #self.t_follow *= interp(self.prev_a[0], [-4, 0], [self.applyDynamicTFollowDecel, 1.0])
      # 선행차감속도* 내차감속도 : 둘다감속이 심하면 더 t_follow를 크게..
      self.t_follow *= interp(radarstate.leadOne.aLeadK, [-4, 0], [self.applyDynamicTFollowDecel, 1.0]) # 선행차의 accel텀은 이미 사용하고 있지만(aLeadK).... 그러나, t_follow에 추가로 적용시험
      self.t_follow *= interp(a_ego, [-4, 0], [self.applyDynamicTFollowDecel, 1.0]) # 내차의 감속도에 추가 적용

      if not self.openpilotLongitudinalControl:
        if v_ego < 0.1:
          self.applyCruiseGap = 1
        elif v_ego_kph < 60:
          self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-5.0, -2.0, -1.0], [4, self.applyCruiseGap, 1]))
        elif v_ego_kph < 120:
          self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-10.0, -5.0, -1.0], [4, self.applyCruiseGap, 2]))
        else:
          self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-20.0, -10.0, -1.0], [4, self.applyCruiseGap, 3]))
      
        #elif a_ego < 0.1:
        #  self.applyCruiseGap = int(interp(a_ego, [-2.0, 0.0], [4, self.applyCruiseGap]))
        #else:
        #  self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [0, 10.0], [self.applyCruiseGap, 1]))

    self.comfort_brake = COMFORT_BRAKE
    self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=lead_xv_0[0,1], v_lead1=lead_xv_1[0,1])

    applyStopDistance = self.stopDistance * (2.0 - mySafeModeFactor)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance, krkeegan=self.applyLongDynamicCost)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance, krkeegan=self.applyLongDynamicCost)
    self.params[:,0] = MIN_ACCEL
    self.params[:,1] = self.max_a

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,5] = self.leadDangerFactor #LEAD_DANGER_FACTOR

      cruiseButtonCounterDiff = controls.cruiseButtonCounter - self.cruiseButtonCounter
      #active_mode => -3(OFF auto), -2(OFF brake), -1(OFF user), 0(OFF), 1(ON user), 2(ON gas), 3(ON auto)
      if controls.longActiveUser <= 0:
        self.e2ePaused = False
      if self.e2ePaused and cruiseButtonCounterDiff != 0: #신호감지무시중 버튼이 눌리면 다시 재개함.
        self.e2ePaused = False
      #if v_ego_kph > 50.0 or self.xState in [XState.lead, XState.cruise] or (v_ego_kph > 30.0 and (model_x > 60.0 and abs(y[-1])<2.0)):
      if self.xState in [XState.lead, XState.cruise] or (v_ego_kph > 30.0 and (model_x > 60.0 and abs(y[-1])<2.0)):
        self.e2ePaused = False

      if controls.myDrivingMode <= 3: #cruiseGap이 1,2,3일때 신호감속.. 4일때는 일반주행.

        #1단계: 모델값을 이용한 신호감지
        model_v = self.vFilter.process(v[-1])
        startSign = model_v > 5.0 or model_v > (v[0]+2)
        if v_ego < 1.0: #정지상태인경우
          stopSign = model_x < 20.0 and model_v < 10.0
        elif v_ego_kph<80.0: # 80키로 이하, 
          stopSign = model_x < 130.0 and ((model_v < 3.0) or (model_v < v[0]*0.70)) and abs(y[N]) < 5.0 #10초후 정지, 70%감속, 직선도로에서 감지 정지...
        else:
          stopSign = False

        ## 현재속도로 정지가 가능한경우에만 신호인식하도록 해보자~, stop_distance는 신호정지시 model_x가 0이므로... 이것도 인지하도록 함.
        self.stopSignCount = self.stopSignCount + 1 if (stopSign and (model_x > get_safe_obstacle_distance(v_ego,t_follow=0, comfort_brake=COMFORT_BRAKE, stop_distance=-1.0))) else 0 
        #self.stopSignCount = self.stopSignCount + 1 if stopSign else 0 

        # cruise_helper에서 깜박이 켜고 신호감지, 브레이크 크루즈ON을 기동하면... 신호오류와 같이, 크루즈버튼으로 출발해야함.
        if self.longActiveUser_prev != controls.longActiveUser:
          if controls.longActiveUser > 10:
            self.trafficError = True
          self.longActiveUser_prev = controls.longActiveUser

        ## 방금 startSign이 잠깐들어오고 StopSign이 들어오면.... 신호감지 오류...
        if 0.0 < self.startSignCount*DT_MDL < 0.1 and stopSign:
          if v_ego < 0.1 and self.xState == XState.e2eStop and not self.e2ePaused:
            #self.trafficError = True  # Traffic Error잠시멈춤..
            pass
        self.startSignCount = self.startSignCount + 1 if startSign else 0

        # trafficState: 2:StartSign, 1:StopSign, 0: Ignore
        # self.trafficState = 1 if self.stopSignCount * DT_MDL > 0.3 else 2 if self.startSignCount * DT_MDL > 0.3 else 0
        # 신호등의 상태를 과거값을 기억하고 있으면 어떨까?

        if self.stopSignCount * DT_MDL > 0.0 and carstate.rightBlinker == False:
           self.trafficState = 1 
        elif self.startSignCount * DT_MDL > 0.1:
           self.trafficState = 2 

        if self.xState == XState.e2eStop: # and abs(self.xStop - model_x) < 20.0:
          stopFilterX = self.xStopFilter.process(model_stop_x, median = True)  # -v_ego는 longitudinalPlan에서 v_ego만큼 더해서 나옴.. 마지막에 급감속하는 문제가 발생..
          self.xStop = self.xStopFilter2.process(stopFilterX)
        else:
          self.xStop = model_stop_x
          self.xStopFilter.set(model_stop_x)
          self.xStopFilter2.set(model_stop_x)
          
        model_x = self.xStop

        if self.e2ePaused:
          self.trafficState |= 100  # 이렇게하면.... 이벤트발생이 안됨...
          self.trafficError = False

        # SOFT_HOLD: 기능
        if carstate.brakePressed and v_ego < 0.1 and self.softHoldMode > 0:  
          self.softHoldTimer += 1
          if self.softHoldTimer*DT_MDL >= 0.7: 
            self.xState = XState.softHold
            self.e2ePaused = False
            self.trafficError = 0
        else:
          self.softHoldTimer = 0

        #2단계: 신호감지조건과 주행조건과의 관계로 상태 결정.
        if self.xState == XState.e2eStop and not self.e2ePaused: 
          if v_ego < 0.1: # 정지상태이면...
            model_x = 0.0
            v_cruise = 0.0
          if radarstate.leadOne.status and (radarstate.leadOne.dRel - model_x) < 2.0:
            self.xState = XState.lead
          elif self.trafficState == 2:
            if not self.trafficError or (self.trafficError and cruiseButtonCounterDiff > 0):  # 출발신호
              self.xState = XState.e2eCruise
              self.e2ePaused = True #출발신호가 나오면 이때부터 신호무시하자... 출발후 정지하는 경우가 생김..
              self.trafficError = False
          if carstate.gasPressed: # or cruiseButtonCounterDiff>0:       #예외: 정지중 accel을 밟으면 강제주행모드로 변경
            self.xState = XState.e2eCruise
            self.e2ePaused = True
        #SOFT_HOLD: 정지 유지상태: 신호오류등 상황발생시 정지유지.
        elif self.xState == XState.softHold: 
          model_x = 0.0
          if carstate.gasPressed:
            self.xState = XState.e2eCruise
            self.e2ePaused = True
          if cruiseButtonCounterDiff > 0:
            self.xState = XState.e2eStop if self.trafficState == 1 else XState.e2eCruise
            self.e2ePaused = False
        #E2E_CRUISE: 주행상태.
        else:
          self.trafficError = False
          if self.status:
            self.xState = XState.lead
          elif self.trafficState == 1 and not self.e2ePaused and not carstate.gasPressed:                 #신호인식이 되면 정지모드
            self.buttonStopDist = 0
            self.xState = XState.e2eStop
          else:
            self.xState = XState.e2eCruise
            if carstate.brakePressed and v_ego_kph < 1.0:  #예외: 정지상태에서 브레이크를 밟으면 강제정지모드.. E2E오류.. SOFT_HOLD
              self.xState = XState.softHold
      else:
        self.xState = XState.cruise
        self.trafficState = 0
        self.trafficError = False

      fakeCruiseDistance = 0.0  #신호정지시: 뒤쪽에서는 문제....

      #3단계: 조건에 따른. 감속및 주행.
      if self.xState in [XState.lead, XState.cruise] or self.e2ePaused or controls.longActiveUser<=0:
        model_x = 1000.0
      elif self.xState == XState.e2eCruise:
        if carstate.gasPressed:
          self.e2ePaused = True
        if model_x > 150.0 or self.e2ePaused or v_ego_kph > self.e2eDecelSpeed:                # 속도가 빠른경우 cruise_obstacle값보다 model_x값이 적어 속도증가(약80키로전후)를 차단함~
          model_x = 1000.0
        #elif self.trafficStopModelSpeed:
        #  v_cruise = v[0]
      elif self.xState == XState.softHold:
        #model_x = stopline_x
        v_cruise = 0
        model_x = 0.0
        pass
      elif self.xState == XState.e2eStop:
        self.comfort_brake = COMFORT_BRAKE * self.trafficStopAccel
        fakeCruiseDistance = 10.0

        #if cruiseButtonCounterDiff > 0:
        #  self.buttonStopDist += 1.0
        #model_x += self.buttonStopDist
        if False: #self.trafficStopModelSpeed:
          v_cruise = v[0]

      self.comfort_brake *= mySafeModeFactor
      self.longActiveUser = controls.longActiveUser
      self.cruiseButtonCounter = controls.cruiseButtonCounter

      stop_x = model_x
      # 급격히 정지하는 걸 막아보자~ 시험.
      #if self.xState == XState.e2eStop and model_x < 3.0: # 신호정지이고 3M이내이면.. 급정거... 최소거리확보해야할... test
      #  stop_x = max(v_ego * v_ego / (1.0 * 2), model_x)  # -1 m/s^2으로 감속할때 정지거리..

      x2 = stop_x * np.ones(N+1) + self.trafficStopDistanceAdjust

      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)

      ## 시험코드: 왜 앞차는 가속하는데... 내차는 가속을 안하냐구...
      ## 결론은 long_mpc에 입력하는 cruise_obstacle값이 작음... 크게하여 가속도를 늘려야함.
      # 레이더있고, 내차보다 빠르면.. 유효거리 50M이내.. 
      # 값을 크게하면... lead_obstacle값으로 가겠지..
      comfort_brake = self.comfort_brake
      if radarstate.leadOne.status and self.applyLongDynamicCost and radarstate.leadOne.dRel < 50:
        comfort_brake *= interp(radarstate.leadOne.vRel*3.6, [0, 2.], [1.0, self.applyDynamicTFollowApart])
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, self.t_follow, comfort_brake, applyStopDistance + fakeCruiseDistance)
      
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])

      #self.debugLongText1 = 'A{:.2f},Y{:.1f},TR={:.2f},state={} {},L{:3.1f} C{:3.1f},{:3.1f},{:3.1f} X{:3.1f} S{:3.1f},V={:.1f}:{:.1f}:{:.1f}'.format(
      #  self.prev_a[0], y[-1], self.t_follow, self.xState, self.e2ePaused, lead_0_obstacle[0], cruise_obstacle[0], cruise_obstacle[1], cruise_obstacle[-1],model.position.x[-1], model_x, v_ego*3.6, v[0]*3.6, v[-1]*3.6)
      self.debugLongText1 = "A{:3.2f},L0{:5.1f},L1{:5.1f},C{:5.1f},X{:5.1f}".format(self.max_a, lead_0_obstacle[0], lead_1_obstacle[0], cruise_obstacle[0], x2[0])

      self.source = SOURCES[np.argmin(x_obstacles[0])]

      if self.source == 'e2e' and model_x < 20.0: #신호감속인경우 그리고 20M이내에는 감속도 제한함.. 정지선 x를 지나가더라도.... 급격한 x값의 변화로 인한 감속정지를 막기위해.... 되려나?
        self.params[:,0] = -COMFORT_BRAKE #MIN_ACCEL  

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = self.t_follow
    self.params[:,6] = self.comfort_brake
    self.params[:,7] = applyStopDistance
    

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.comfort_brake, self.stopDistance)) - self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.comfort_brake, self.stopDistance)) - self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

    self.v_cruise = v_cruise
    self.x_obstacle_min = self.params[:,2]

  def run(self):
    # t0 = sec_since_boot()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + 0.05, T_IDXS, self.a_solution)

    t = sec_since_boot()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(sec_since_boot() - t0):.2e} qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
