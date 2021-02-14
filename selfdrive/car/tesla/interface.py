#!/usr/bin/env python3
from cereal import car
from selfdrive.car.tesla.values import CAR
from selfdrive.car import STD_CARGO_KG, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    # TODO: is this correct?
    return accel

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "tesla"
    ret.safetyModel = car.CarParams.SafetyModel.tesla

    if candidate == CAR.AP2_MODELS:
      ret.mass = 2100. + STD_CARGO_KG
      ret.rotationalInertia = 2500.
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 11.5
      ret.tireStiffnessFront = 85100
      ret.tireStiffnessRear = 90000
    else:
      raise ValueError(f"Unsupported car: {candidate}")

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    events = self.create_common_events(ret)

    ret.events = events.to_msg()

    return ret.as_reader()

  def apply(self, c):
    # No control for now
    return []
