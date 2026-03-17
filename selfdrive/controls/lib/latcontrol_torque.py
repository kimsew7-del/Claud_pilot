import math
import numpy as np
from collections import deque

from cereal import log
from common.numpy_fast import interp
from common.filter_simple import FirstOrderFilter
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

from common.params import Params
from decimal import Decimal

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

# Backported from master: improved low speed factor via speed-dependent KP,
# jerk-based feedforward for smoother transitions, and delay-aware setpoint.

FRICTION_THRESHOLD = 0.2

# Master-style speed-dependent KP (replaces old flat low_speed_factor)
KP_DEFAULT = 0.8
KI_DEFAULT = 0.15
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP_DEFAULT]

# Jerk feedforward parameters (backported from master)
LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0

DT_CTRL = 0.01  # 100Hz control loop


def get_friction(error, lateral_accel_deadzone, friction_threshold, friction_coeff):
  """Compute friction compensation (backported from master opendbc.car.lateral)"""
  error_deadzone = apply_deadzone(error, lateral_accel_deadzone)
  friction_interp = interp(error_deadzone, [-friction_threshold, friction_threshold],
                           [-friction_coeff, friction_coeff])
  return friction_interp


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.CP = CP

    self.mpc_frame = 0
    self.params = Params()

    self.kf = CP.lateralTuning.torque.kf

    # Use speed-dependent KP from master instead of flat kp
    # The PID operates in lateral acceleration space
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI_DEFAULT,
                             k_f=self.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    self.steering_angle_deadzone_deg = CP.lateralTuning.torque.steeringAngleDeadzoneDeg

    self.live_tune_enabled = False
    self.lt_timer = 0

    # Jerk feedforward state (backported from master)
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / DT_CTRL)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len,
                                          maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / DT_CTRL)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), DT_CTRL)

  def live_tune(self, CP):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      self.max_lat_accel = float(Decimal(self.params.get("TorqueMaxLatAccel", encoding="utf8")) * Decimal('0.1'))
      self.kp = float(Decimal(self.params.get("TorqueKp", encoding="utf8")) * Decimal('0.1')) / self.max_lat_accel
      self.kf = float(Decimal(self.params.get("TorqueKf", encoding="utf8")) * Decimal('0.1')) / self.max_lat_accel
      self.ki = float(Decimal(self.params.get("TorqueKi", encoding="utf8")) * Decimal('0.1')) / self.max_lat_accel
      self.friction = float(Decimal(self.params.get("TorqueFriction", encoding="utf8")) * Decimal('0.001'))
      self.use_steering_angle = self.params.get_bool('TorqueUseAngle')
      self.steering_angle_deadzone_deg = float(Decimal(self.params.get("TorqueAngDeadZone", encoding="utf8")) * Decimal('0.1'))
      # When live tuning, override PID gains but keep speed-dependent KP structure
      self.pid = PIDController(self.kp, self.ki,
                              k_f=self.kf, pos_limit=1.0, neg_limit=-1.0)

      self.mpc_frame = 0

  def update(self, active, CS, CP, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
    self.lt_timer += 1
    if self.lt_timer > 100:
      self.lt_timer = 0
      self.live_tune_enabled = self.params.get_bool("OpkrLiveTunePanelEnable")
    if self.live_tune_enabled:
      self.live_tune(CP)

    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      # Still update buffer even when inactive for smooth transition
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0

      # Compute lateral accelerations
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      # Backported from master: delay-aware setpoint using request buffer
      self.lat_accel_request_buffer.append(desired_lateral_accel)

      # Use a simple delay estimate (typical steering actuator delay ~0.1s)
      lat_delay_estimate = 0.1
      delay_frames = int(np.clip(lat_delay_estimate / DT_CTRL + 1, 1, self.lat_accel_request_buffer_len))
      setpoint = self.lat_accel_request_buffer[-delay_frames]
      measurement = actual_lateral_accel
      error = setpoint - measurement

      # Backported from master: jerk-based feedforward for smoother transitions
      lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames,
                                  -self.lat_accel_request_buffer_len + 1, -2))
      raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx + 1] -
                          self.lat_accel_request_buffer[lookahead_idx - 1]) / (2 * DT_CTRL)
      desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)

      pid_log.error = error

      # Feedforward: gravity-compensated desired lateral accel + friction + jerk
      ff = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      # Backported friction: use error + jerk for anticipatory friction compensation
      friction_compensation = get_friction(error + JERK_GAIN * desired_lateral_jerk,
                                           lateral_accel_deadzone,
                                           FRICTION_THRESHOLD, self.friction)
      ff += friction_compensation / self.kf

      freeze_integrator = CS.steeringRateLimited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
