from common.numpy_fast import clip, interp
from selfdrive.car.tesla.teslacan import TeslaCAN
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import CarControllerParams

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.last_angle = 0
    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(dbc_name, self.packer)

  def update(self, enabled, CS, frame, actuators, cruise_cancel):
    can_sends = []

    if enabled:
      # Angular rate limit based on speed
      apply_angle = actuators.steeringAngleDeg
      steer_up = (self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle))
      rate_limit = CarControllerParams.RATE_LIMIT_UP if steer_up else CarControllerParams.RATE_LIMIT_DOWN
      max_angle_diff = interp(CS.out.vEgo, rate_limit.speed_points, rate_limit.max_angle_diff_points)
      apply_angle = clip(apply_angle, (self.last_angle - max_angle_diff), (self.last_angle + max_angle_diff))

      # TODO: Driver torque blending
    else:
      apply_angle = CS.out.steeringAngleDeg
    self.last_angle = apply_angle

    can_sends.append(self.tesla_can.create_steering_control(apply_angle, enabled, frame))

    # Cancel when openpilot is not enabled anymore
    if not enabled and bool(CS.out.cruiseState.enabled):
      cruise_cancel = True

    # TODO: check action request receive time and send on change instead of 10 Hz
    if ((frame % 10) == 0):
      can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, cruise_cancel))

    # TODO: HUD control

    return can_sends
