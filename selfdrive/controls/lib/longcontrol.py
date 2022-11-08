from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.modeld.constants import T_IDXS
from common.params import Params

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill, softHold):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
               (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

    if softHold:
      long_control_state = LongCtrlState.stopping
  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.debugLoCText = ""
    self.readParamCount = 0
    self.longitudinalActuatorDelayUpperBoundCost = 1.0
    self.longitudinalActuatorDelayLowerBoundCost = 1.0
    self.longitudinalTuningKf = 1.0
    self.longitudinalPlanFF = 0.0
    self.accelBoost = 1.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan, CC):
    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
      self.longitudinalActuatorDelayUpperBoundCost = float(int(Params().get("LongitudinalActuatorDelayUpperBound", encoding="utf8"))) / 100.
      self.longitudinalActuatorDelayLowerBoundCost = float(int(Params().get("LongitudinalActuatorDelayLowerBound", encoding="utf8"))) / 100.
      self.longitudinalTuningKf = float(int(Params().get("LongitudinalTuningKf", encoding="utf8"))) / 100.
      self.longitudinalPlanFF = float(int(Params().get("LongitudinalPlanFF", encoding="utf8"))) / 100.
      self.accelBoost = float(int(Params().get("AccelBoost", encoding="utf8"))) / 100.

      
    t_since_plan += self.longitudinalPlanFF

    longitudinalActuatorDelayLowerBound = self.CP.longitudinalActuatorDelayLowerBound * 1.0 #self.longitudinalActuatorDelayLowerBoundCost
    longitudinalActuatorDelayUpperBound = self.CP.longitudinalActuatorDelayUpperBound * 1.0 #self.longitudinalActuatorDelayUpperBoundCost
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)
      if a_target < 0.0:
        a_target *= self.longitudinalTuningKf

      v_target_1sec = interp(longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, T_IDXS[:CONTROL_N], speeds)

    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1] * self.accelBoost

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill, CC.hudControl.softHold)
    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1] * self.accelBoost)

    self.debugLoCText = "T:{:.2f},{:.2f} V:{:.2f}={:.1f}-{:.1f} Aout:{:.2f}<{:.2f}".format(t_since_plan, longitudinalActuatorDelayLowerBound, (self.v_pid - CS.vEgo)*3.6, self.v_pid*3.6, CS.vEgo*3.3, self.last_output_accel, output_accel)
    return self.last_output_accel
