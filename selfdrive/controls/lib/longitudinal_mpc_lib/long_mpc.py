#!/usr/bin/env python3
import os
import numpy as np

from common.realtime import sec_since_boot
from common.numpy_fast import clip, interp
from system.swaglog import cloudlog
from selfdrive.modeld.constants import index_function
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from common.conversions import Conversions as CV
from common.params import Params
from common.realtime import DT_MDL

if __name__ == '__main__':  # generating code
  from pyextra.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
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
CRASH_DISTANCE = .5
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


AUTO_TR_BP = [0., 50.*CV.KPH_TO_MS, 100.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
#AUTO_TR_V = [1.0, 1.2, 1.35, 1.45]
AUTO_TR_V = [1.2, 1.2, 1.3, 1.40]
# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
MIN_ACCEL = -3.5
MAX_ACCEL = 2.0
T_FOLLOW = 1.45
COMFORT_BRAKE = 2.3 #2.5
STOP_DISTANCE = 6.5

def get_stopped_equivalence_factor(v_lead, v_ego, t_follow=T_FOLLOW, stop_distance=STOP_DISTANCE):
  return (v_lead**2) / (2 * COMFORT_BRAKE)
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
    self.e2eMode = False #ajouatom
    self.onStopping = False
    self.debugText = ""
    self.trafficState = 0
    self.debugLong = 0
    self.XEgoObstacleCost = 3.
    self.JEgoCost = 5.
    self.AChangeCost = 200.
    self.DangerZoneCost = 100.
    self.trafficStopDistanceAdjust = 0.
    self.applyLongDynamicCost = False
    self.trafficStopAccel = 1.
    self.stopDistance = STOP_DISTANCE
    self.softHoldTimer = 0
    self.lo_timer = 0 

    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = "CRUISE"
    self.e2ePaused = False
    self.longActiveUser = 0
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

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
    self.xState = "CRUISE"
    self.startSignCount = 0
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
    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) # 가까울수록 작게 < 이것이 민감하다는 것인가?
    j_ego_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) #가까울수록 작게 <- 이것이 민감하다는 것인가?
    d_zone_tf = interp(self.t_follow, TFs, [1.3, 1., 1.]) # 가까울수록 크게 <- 민감?

# 전방차량과의 상대속도에 의한 j,a cost변경
    # KRKeegan adjustments to improve sluggish acceleration
    # do not apply to deceleration
    j_ego_v_ego = 1
    a_change_v_ego = 1
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):
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
    self.cruise_min_a = min_a
    self.cruise_max_a = max_a

  def update(self, carstate, radarstate, model, controls, v_cruise, x, v, a, j, y, prev_accel_constraint):
    v_ego = self.x0[1]
    self.lo_timer += 1
    if self.lo_timer > 100:
      self.lo_timer = 0
      self.XEgoObstacleCost = float(int(Params().get("XEgoObstacleCost", encoding="utf8")))
      self.JEgoCost = float(int(Params().get("JEgoCost", encoding="utf8")))
      self.AChangeCost = float(int(Params().get("AChangeCost", encoding="utf8")))
      self.DangerZoneCost = float(int(Params().get("DangerZoneCost", encoding="utf8")))
      self.trafficStopDistanceAdjust = float(int(Params().get("TrafficStopDistanceAdjust", encoding="utf8"))) / 100.
      self.applyLongDynamicCost = Params().get_bool("ApplyLongDynamicCost")
      self.trafficStopAccel = float(int(Params().get("TrafficStopAccel", encoding="utf8"))) / 100.
      self.stopDistance = float(int(Params().get("StopDistance", encoding="utf8"))) / 100.
    self.debugLong = 0
    self.trafficState = 0

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    
    self.t_follow = interp(carstate.vEgo, AUTO_TR_BP, AUTO_TR_V) if self.mode == 'acc' else T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=lead_xv_0[0,1], v_lead1=lead_xv_1[0,1])

    #leadObstacle, cruiseObstacle은 self.stopDistance로 계산.
    #traffic에 대한 x2는 applyStopDistance적용..
    #leadObstacle, cruiseObstacle, xObstacle중 가작 장은값적용할때....
    # "E2E_STOP" 즉 신호정지 인경우에만... applyStopDistance를 trafficStopDistanceAdjust값을 더하고, solver로 보냄...
    # solver에서는 이 stopdistance로 계산~
    # x값에는 정지선에대한 보정값을 넣음...
    # 맞게 생각한건지.....
    applyStopDistance = self.stopDistance

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance)

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,0] = MIN_ACCEL
      self.params[:,1] = self.cruise_max_a
      self.params[:,5] = LEAD_DANGER_FACTOR

      model_x = x[N] 
      longActiveUserChanged = 0
      #active_mode => -3(OFF auto), -2(OFF brake), -1(OFF user), 0(OFF), 1(ON user), 2(ON gas), 3(ON auto)
      if controls.longActiveUser != self.longActiveUser:
        longActiveUserChanged = controls.longActiveUser
      self.longActiveUser = controls.longActiveUser
      if v_ego*CV.MS_TO_KPH > 50.0 or longActiveUserChanged<0 or self.xState in ["LEAD", "CRUISE"] or v_ego*CV.MS_TO_KPH > 30.0 and (model_x > 50.0 and abs(y[N])<3.0):
        self.e2ePaused = False

      if self.e2eMode:
        startSign = v[-1] > 5.0
        #stopSign = model_x < 120.0 and ((v[-1] < 3.0) or (v[-1] < v_ego*0.95)) and abs(y[N]) < 3.0 #직선도로에서만 감지하도록 함~
        stopSign = ((v[-1] < 3.0) or (v[-1] < v_ego*0.60)) and abs(y[N]) < 20.0 #직선도로에서만 감지하도록 함~
        self.trafficState = 1 if stopSign else 2 if startSign else 0 
        if startSign:
          self.startSignCount = self.startSignCount + 1 #모델은 0.05초  /1 frame
        else:
          self.startSignCount = 0

        #E2E_STOP: 감속정지상태, 정지선 밖(2M이상)에 차량이 있어도 무시, 상태유지: 정지상태에서는 전방에 리드가 감지되어도 정지해야함. 
        if self.xState == "E2E_STOP" and not self.e2ePaused: 
          if v_ego < 0.1: # 정지상태이면...
            model_x = -5.0
          if radarstate.leadOne.status and (radarstate.leadOne.dRel - model_x) < 2.0:
            self.xState = "LEAD"
          elif self.startSignCount*DT_MDL >= 0.5: #10*0.05 => 0.5초
            self.xState = "E2E_CRUISE"
          if carstate.brakePressed and v_ego < 0.1:  #예외: 정지상태에서 브레이크를 밟으면 강제정지모드.. E2E오류.. E2E_STOP2
            self.softHoldTimer += 1
            if self.softHoldTimer*DT_MDL >= 1.5: 
              self.xState = "E2E_STOP2"
              self.e2ePaused = False
          else:
            self.softHoldTimer = 0
          if carstate.gasPressed or longActiveUserChanged==1:       #예외: 정지중 accel을 밟으면 강제주행모드로 변경
            self.xState = "E2E_CRUISE"
            self.e2ePaused = True
        #E2E_STOP2: 정지 유지상태: 신호오류등 상황발생시 정지유지.
        elif self.xState == "E2E_STOP2": 
          model_x = -5.0
          if carstate.gasPressed or longActiveUserChanged==1:
            self.xState = "E2E_CRUISE"
        #E2E_CRUISE: 주행상태.
        else:
          if self.status:
            self.xState = "LEAD"
          elif stopSign and not self.e2ePaused:                 #신호인식이 되면 정지모드
            self.xState = "E2E_STOP"
          else:
            self.xState = "E2E_CRUISE"
          if carstate.brakePressed and v_ego*CV.MS_TO_KPH < 1.0:  #예외: 정지상태에서 브레이크를 밟으면 강제정지모드.. E2E오류.. E2E_STOP2
            self.xState = "E2E_STOP2"
      else:
        self.xState = "CRUISE"

      if self.xState in ["LEAD", "CRUISE"] or self.e2ePaused:
        model_x = 400.0
      elif self.xState == "E2E_CRUISE":
        if model_x > 150.0 or self.e2ePaused:                # 속도가 빠른경우 cruise_obstacle값보다 model_x값이 적어 속도증가(약80키로전후)를 차단함~
          model_x = 400.0
        elif model_x > 100.0:
          self.e2ePaused = False
      elif self.xState == "E2E_STOP2":
        #model_x = stopline_x
        pass
      elif self.xState == "E2E_STOP":
        self.comfort_brake = COMFORT_BRAKE * self.trafficStopAccel
        applyStopDistance = self.trafficStopDistanceAdjust

      x2 = model_x * np.ones(N+1)

      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.cruise_max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, self.t_follow, self.comfort_brake, self.stopDistance)
      
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])

      str1 = 'Y{:.1f},TR={:.2f},state={} {},L{:3.1f} C{:3.1f} X{:3.1f} S{:3.1f},V={:.1f}:{:.1f}:{:.1f}:{:.1f}'.format(
        y[N],self.t_follow, self.xState, self.e2ePaused, lead_0_obstacle[0], cruise_obstacle[0], x[N], model_x, v_ego, v[0], v[1], v[-1])
      self.debugText = str1

      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,0] = MIN_ACCEL
      self.params[:,1] = MAX_ACCEL

      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[0,0] < x_and_cruise[0,1] else 'cruise'

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
    if (np.any(lead_xv_0[:,0] - self.x_sol[:,0] < CRASH_DISTANCE) and
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
