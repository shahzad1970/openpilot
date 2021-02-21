from cereal import car
from selfdrive.car.tesla.values import DBC, CANBUS, CRUISESTATE, SPEEDUNITS, GEAR, BUTTONS
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.button_states = {button.event_type: False for button in BUTTONS}

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    ret.gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0
    ret.gasPressed = (ret.gas > 0)

    # Brake pedal
    ret.brake = 0 # TODO: find if this exists
    ret.brakePressed = bool(cp.vl["DI_torque2"]["DI_brakePedal"])

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP"]
    ret.steeringRateDeg = cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = cp.vl["EPAS_sysStatus"]["EPAS_torsionBarTorque"]
    ret.steeringPressed = (cp.vl["EPAS_sysStatus"]["EPAS_handsOnLevel"] > 0)
    ret.steerError = bool(cp.vl["EPAS_sysStatus"]["EPAS_steeringFault"])

    # Cruise state
    ret.cruiseState.enabled = (cp.vl["DI_state"]["DI_cruiseState"] in [CRUISESTATE.ENABLED, CRUISESTATE.STANDSTILL, CRUISESTATE.OVERRIDE, CRUISESTATE.PRE_FAULT, CRUISESTATE.PRE_CANCEL])
    if cp.vl["DI_state"]["DI_speedUnits"] == SPEEDUNITS.KPH:
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
    elif cp.vl["DI_state"]["DI_speedUnits"] == SPEEDUNITS.MPH:
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
    ret.cruiseState.available = ((cp.vl["DI_state"]["DI_cruiseState"] == CRUISESTATE.STANDBY) or ret.cruiseState.enabled)
    ret.cruiseState.standstill = (cp.vl["DI_state"]["DI_cruiseState"] == CRUISESTATE.STANDSTILL)

    # Gear
    ret.gearShifter = GEAR.GEAR_MAP[cp.vl["DI_torque2"]["DI_gear"]]

    # Buttons
    buttonEvents = []
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        buttonEvents.append(event)
      self.button_states[button.event_type] = state
    ret.buttonEvents = buttonEvents

    # Doors
    # TODO: convert constants to DBC values
    door_fl_open = (cp.vl["GTW_carState"]["DOOR_STATE_FL"] == 1)
    door_fr_open = (cp.vl["GTW_carState"]["DOOR_STATE_FR"] == 1)
    door_rl_open = (cp.vl["GTW_carState"]["DOOR_STATE_RL"] == 1)
    door_rr_open = (cp.vl["GTW_carState"]["DOOR_STATE_RR"] == 1)
    door_frunk_open = (cp.vl["GTW_carState"]["DOOR_STATE_FrontTrunk"] == 1)
    door_boot_open = (cp.vl["GTW_carState"]["BOOT_STATE"] == 1)
    ret.doorOpen = (door_fl_open or door_fr_open or door_rl_open or door_rr_open or door_frunk_open or door_boot_open)

    # Blinkers
    # TODO: convert constants to DBC values
    ret.leftBlinker = (cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1)
    ret.rightBlinker = (cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1)

    # Seatbelt
    # TODO: convert constants to DBC values
    ret.seatbeltUnlatched = (cp.vl["SDM1"]["SDM_bcklDrivStatus"] == 1)

    # TODO: blinkers, blindspot

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("ESP_vehicleSpeed", "ESP_B", 0),
      ("DI_pedalPos", "DI_torque1", 0),
      ("DI_brakePedal", "DI_torque2", 0),
      ("StW_AnglHP", "STW_ANGLHP_STAT", 0),
      ("StW_AnglHP_Spd", "STW_ANGLHP_STAT", 0),
      ("EPAS_handsOnLevel", "EPAS_sysStatus", 0),
      ("EPAS_torsionBarTorque", "EPAS_sysStatus", 0),
      ("EPAS_steeringFault", "EPAS_sysStatus", 0),
      ("DI_cruiseState", "DI_state", 0),
      ("DI_digitalSpeed", "DI_state", 0),
      ("DI_speedUnits", "DI_state", 0),
      ("DI_gear", "DI_torque2", 0),
      ("SpdCtrlLvr_Stat", "STW_ACTN_RQ", 0),
      ("TurnIndLvr_Stat", "STW_ACTN_RQ", 0),
      ("DOOR_STATE_FL", "GTW_carState", 1),
      ("DOOR_STATE_FR", "GTW_carState", 1),
      ("DOOR_STATE_RL", "GTW_carState", 1),
      ("DOOR_STATE_RR", "GTW_carState", 1),
      ("DOOR_STATE_FrontTrunk", "GTW_carState", 1),
      ("BOOT_STATE", "GTW_carState", 1),
      ("BC_indicatorLStatus", "GTW_carState", 1),
      ("BC_indicatorRStatus", "GTW_carState", 1),
      ("SDM_bcklDrivStatus", "SDM1"),
    ]

    checks = [
      # sig_address, frequency
      ("ESP_B", 50),
      ("DI_torque1", 100),
      ("DI_torque2", 100),
      ("STW_ANGLHP_STAT", 100),
      ("EPAS_sysStatus", 25),
      ("DI_state", 10),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),
      ("SDM1", 10),
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, CANBUS.chassis)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default

    ]
    checks = [
      # sig_address, frequency

    ]
    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, CANBUS.autopilot)
