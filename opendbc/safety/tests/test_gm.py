#!/usr/bin/env python3
import unittest
from collections.abc import Callable

from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class Buttons:
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  CANCEL = 6


class GmLongitudinalBase(common.CarSafetyTest, common.LongitudinalGasBrakeSafetyTest):

  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB), 2: (0x184,)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd, PSCMStatus

  MAX_POSSIBLE_BRAKE = 2 ** 12
  MAX_BRAKE = 400

  MAX_POSSIBLE_GAS = 4000  # reasonably excessive limits, not signal max
  MIN_POSSIBLE_GAS = -4000

  PCM_CRUISE = False  # openpilot can control the PCM state if longitudinal

  BRAKE_BUS: int
  packer_chassis: CANPackerSafety
  _button_msg: Callable[..., libsafety_py.CANPacket]

  def _send_brake_msg(self, brake):
    values = {"FrictionBrakeCmd": -brake}
    return self.packer_chassis.make_can_msg_safety("EBCMFrictionBrakeCmd", self.BRAKE_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"GasRegenCmd": gas}
    return self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)

  def test_gas_apply(self):
    # apply bit is only allowed with controls allowed
    values = {"GasRegenCmd": self.INACTIVE_GAS, "GasRegenCmdActive": 1}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)))

  # override these tests from CarSafetyTest, GM longitudinal uses button enable
  def _pcm_status_msg(self, enable):
    raise NotImplementedError

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling and rising edges, respectively.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        with self.subTest(btn_prev=btn_prev, btn_cur=btn_cur):
          self._rx(self._button_msg(btn_prev))
          self.safety.set_controls_allowed(0)
          for _ in range(10):
            self._rx(self._button_msg(btn_cur))

          should_enable = btn_cur != Buttons.DECEL_SET and btn_prev == Buttons.DECEL_SET
          should_enable = should_enable or (btn_cur == Buttons.RES_ACCEL and btn_prev != Buttons.RES_ACCEL)
          should_enable = should_enable and btn_cur != Buttons.CANCEL
          self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())


class TestGmSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "gm_global_a_powertrain_generated"
  SAFETY_MODEL = CarParams.SafetyModel.gm

  STANDSTILL_THRESHOLD = 10 * 0.0311
  # Ensures ASCM is off on ASCM cars, and relay is not malfunctioning for camera-ACC cars
  RELAY_MALFUNCTION_ADDRS = {0: (0x180,), 2: (0x184,)}  # ASCMLKASteeringCmd, PSCMStatus
  BUTTONS_BUS = 0  # rx or tx
  BRAKE_BUS = 0  # tx only

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 15
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 128
  DRIVER_TORQUE_ALLOWANCE = 65
  DRIVER_TORQUE_FACTOR = 4

  PCM_CRUISE = True  # openpilot is tied to the PCM state if not longitudinal

  EXTRA_SAFETY_PARAM = 0

  def setUp(self):
    super().setUp()
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")

  def _pcm_status_msg(self, enable):
    if self.PCM_CRUISE:
      values = {"CruiseState": enable}
      return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)
    else:
      raise NotImplementedError

  def _speed_msg(self, speed, wheels=("RL", "RR")):
    values = {"%sWheelSpd" % s: speed for s in wheels}
    return self.packer.make_can_msg_safety("EBCMWheelSpdRear", 0, values)

  def _user_brake_msg(self, brake):
    # GM safety has a brake threshold of 8
    values = {"BrakePedalPos": 8 if brake else 0}
    return self.packer.make_can_msg_safety("ECMAcceleratorPos", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal2": 1 if gas else 0}
    if self.PCM_CRUISE:
      # Fill CruiseState with expected value if the safety mode reads cruise state from gas msg
      values["CruiseState"] = self.safety.get_controls_allowed()
    return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)

  def _torque_driver_msg(self, torque):
    # Safety tests assume driver torque is an int, use DBC factor
    values = {"LKADriverAppldTrq": torque * 0.01}
    return self.packer.make_can_msg_safety("PSCMStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKASteeringCmd": torque, "LKASteeringCmdActive": steer_req}
    return self.packer.make_can_msg_safety("ASCMLKASteeringCmd", 0, values)

  def _button_msg(self, buttons):
    values = {"ACCButtons": buttons}
    return self.packer.make_can_msg_safety("ASCMSteeringButton", self.BUTTONS_BUS, values)

  def test_vehicle_moving_single_wheel(self):
    # vehicle is moving if any checked wheel is above standstill
    for wheel in ("RL", "RR"):
      self.assertTrue(self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1, wheels=(wheel,))))
      self.assertTrue(self.safety.get_vehicle_moving())


class TestGmEVSafetyBase(TestGmSafetyBase):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV

  # existence of _user_regen_msg adds regen tests
  def _user_regen_msg(self, regen):
    values = {"RegenPaddle": 2 if regen else 0}
    return self.packer.make_can_msg_safety("EBCMRegenPaddle", 0, values)


class TestGmAscmSafety(GmLongitudinalBase, TestGmSafetyBase):
  TX_MSGS = [[0x180, 0], [0x409, 0], [0x40A, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0xA1, 1], [0x306, 1], [0x308, 1], [0x310, 1],  # obs bus
             [0x315, 2]]  # ch bus
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd
  FWD_BUS_LOOKUP: dict[int, int] = {}
  BRAKE_BUS = 2

  MAX_GAS = 1018
  MIN_GAS = -650  # maximum regen
  INACTIVE_GAS = -650

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM


class TestGmAscmEVSafety(TestGmAscmSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraSafetyBase(TestGmSafetyBase):
  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("ECMEngineStatus", 0, values)


class TestGmCameraSafety(TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180], 0: [0x184]}  # block LKAS message and PSCMStatus
  BUTTONS_BUS = 2  # tx only

  @property
  def SAFETY_PARAM(self):
    return GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM

  def test_rx_buttons_do_not_enable(self):
    self._rx(self.packer.make_can_msg_safety("ASCMSteeringButton", 0, {"ACCButtons": Buttons.RES_ACCEL}))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_buttons(self):
    # Only CANCEL button is allowed while cruise is enabled
    self.safety.set_controls_allowed(0)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    self.safety.set_controls_allowed(1)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL)))


class TestGmCameraEVSafety(TestGmCameraSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraLongitudinalSafety(GmLongitudinalBase, TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0], [0x315, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180, 0x2CB, 0x370, 0x315], 0: [0x184]}  # block LKAS, ACC messages and PSCMStatus
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB, 0x370, 0x315), 2: (0x184,)}
  BUTTONS_BUS = 0  # rx only

  MAX_GAS = 1346
  MIN_GAS = -540  # maximum regen
  INACTIVE_GAS = -500

  @property
  def SAFETY_PARAM(self):
    return GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | self.EXTRA_SAFETY_PARAM


class TestGmSilveradoCameraLongitudinalSafety(TestGmCameraLongitudinalSafety):
  # The raised torque envelope must never affect the standard GM camera configurations.
  MAX_GAS = 2450

  @property
  def SAFETY_PARAM(self):
    return super().SAFETY_PARAM | GMSafetyFlags.SILVERADO_TORQUE

  def test_silverado_torque_requires_camera_longitudinal_flags(self):
    for safety_param in (GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG,
                         GMSafetyFlags.HW_CAM | GMSafetyFlags.SILVERADO_TORQUE,
                         GMSafetyFlags.SILVERADO_TORQUE):
      with self.subTest(safety_param=safety_param):
        self.safety.set_safety_hooks(self.SAFETY_MODEL, safety_param)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._send_gas_msg(2450)))


class TestGmSilveradoLateralOnlySafety(TestGmSilveradoCameraLongitudinalSafety):
  @property
  def SAFETY_PARAM(self):
    return super().SAFETY_PARAM | GMSafetyFlags.LATERAL_ONLY

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("ECMPRDNL2", 0, {"PRNDL2": gear})

  def _fresh_inputs(self, gear=4):
    for name, values in (("PSCMStatus", {}), ("EBCMWheelSpdRear", {}),
                         ("ASCMSteeringButton", {"ACCButtons": Buttons.UNPRESS}),
                         ("ECMAcceleratorPos", {}), ("AcceleratorPedal2", {}),
                         ("ECMEngineStatus", {}), ("ECMPRDNL2", {"PRNDL2": gear})):
      self.assertTrue(self._rx(self.packer.make_can_msg_safety(name, 0, values)))

  def _enable_lateral(self):
    self._fresh_inputs()
    self.safety.safety_set_lateral_request(True, True)
    self.assertTrue(self.safety.safety_a.state.lateral_allowed)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lateral_boot_requires_intent_readiness_and_drive(self):
    for request, enabled, gear in ((False, False, 4), (False, True, 4), (True, False, 4),
                                   (True, True, 0), (True, True, 1), (True, True, 2),
                                   (True, True, 3), (True, True, 5), (True, True, 6), (True, True, 7)):
      with self.subTest(request=request, enabled=enabled, gear=gear):
        self._reset_safety_hooks()
        self._fresh_inputs(gear)
        self.safety.safety_set_lateral_request(request, enabled)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self._reset_safety_hooks()
    self._enable_lateral()
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_heartbeat_flags_legacy_and_unknown(self):
    for flags in (0, 1, 2, 3):
      with self.subTest(flags=flags):
        self._reset_safety_hooks()
        self._fresh_inputs()
        self.safety.safety_set_lateral_request_flags(flags)
        self.assertEqual(self._tx(self._torque_cmd_msg(1)), flags == 3)
        self.assertFalse(self.safety.get_controls_allowed())
    for flags in (8, 9, 15, 255, 256, 32768, 65535):
      with self.subTest(flags=flags):
        self._reset_safety_hooks()
        self._enable_lateral()
        self.safety.safety_set_lateral_request_flags(flags)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
        self.assertTrue(self.safety.safety_a.state.lateral_suspended)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
        self.safety.safety_set_lateral_request_flags(0)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_host_fault_during_pause_survives_host_restart(self):
    for pause in ('brake', 'reverse', 'not_ready'):
      for fault_flags in (4, 5, 6, 7):
        with self.subTest(pause=pause, fault_flags=fault_flags):
          self._reset_safety_hooks()
          self._enable_lateral()
          if pause == 'brake':
            self._rx(self._user_brake_msg(True))
          elif pause == 'reverse':
            self._rx(self._gear_msg(2))
          else:
            self.safety.safety_set_lateral_request_flags(2)
          self.safety.safety_set_lateral_request_flags(fault_flags)
          self.assertFalse(self._tx(self._torque_cmd_msg(1)))
          self.assertTrue(self.safety.safety_a.state.lateral_suspended)
          self.assertTrue(self.safety.safety_a.state.lateral_enabled)
          self._fresh_inputs()
          # A restarted host has lost its software latch but cannot clear Panda's.
          self.safety.safety_set_lateral_request_flags(3)
          self.assertFalse(self._tx(self._torque_cmd_msg(1)))
          self.safety.safety_set_lateral_request_flags(0)
          self.safety.safety_set_lateral_request_flags(3)
          self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_startup_host_fault_waits_without_latching(self):
    self._fresh_inputs()
    self.safety.safety_set_lateral_request_flags(6)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self.safety.safety_a.state.lateral_suspended)
    self.safety.safety_set_lateral_request_flags(3)
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_physical_rearm_tolerates_old_host_fault_snapshot(self):
    for button in (Buttons.DECEL_SET, Buttons.RES_ACCEL):
      with self.subTest(button=button):
        self._reset_safety_hooks()
        self._enable_lateral()
        self.safety.safety_set_lateral_request_flags(6)
        self.assertTrue(self.safety.safety_a.state.lateral_suspended)
        self._rx(self._button_msg(button))
        self._rx(self._button_msg(Buttons.UNPRESS))
        self.safety.set_controls_allowed(False)
        self.safety.set_timer(100000)
        self._fresh_inputs()
        self.safety.safety_set_lateral_request_flags(6)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
        self.assertFalse(self.safety.safety_a.state.lateral_suspended)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_physical_rearm_host_fault_grace_expires(self):
    for start in (0, 0xFFFFFF00):
      with self.subTest(start=start):
        self._reset_safety_hooks()
        self.safety.set_timer(start)
        self._enable_lateral()
        self._rx(self._button_msg(Buttons.RES_ACCEL))
        self._rx(self._button_msg(Buttons.UNPRESS))
        self.safety.set_controls_allowed(False)
        for elapsed, suspended in ((499999, False), (500000, True)):
          self.safety.set_timer((start + elapsed) & 0xFFFFFFFF)
          self._fresh_inputs()
          self.safety.safety_set_lateral_request_flags(6)
          self.assertFalse(self._tx(self._torque_cmd_msg(1)))
          self.assertEqual(self.safety.safety_a.state.lateral_suspended, suspended)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_physical_rearm_never_masks_independent_fault_or_cancel(self):
    for fault in ('invalid_gear', 'relay', 'cancel', 'unknown_flags', 'stale_can'):
      with self.subTest(fault=fault):
        self._reset_safety_hooks()
        self._enable_lateral()
        self._rx(self._button_msg(Buttons.RES_ACCEL))
        self._rx(self._button_msg(Buttons.UNPRESS))
        self.safety.set_controls_allowed(False)
        self.safety.set_timer(100000)
        if fault == 'invalid_gear':
          self._rx(self._gear_msg(15))
        elif fault == 'relay':
          self.safety.set_relay_malfunction(True)
        elif fault == 'cancel':
          self._rx(self._button_msg(Buttons.CANCEL))
        elif fault == 'unknown_flags':
          self.safety.safety_set_lateral_request_flags(8)
        else:
          self.safety.set_rx_checks_received(0, 500000)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
        self.assertTrue(self.safety.safety_a.state.lateral_suspended)

  def test_lateral_does_not_grant_longitudinal(self):
    self._enable_lateral()
    self.assertFalse(self._tx(self._send_gas_msg(1)))
    self.assertFalse(self._tx(self._send_brake_msg(1)))
    self.assertTrue(self._tx(self._send_gas_msg(self.INACTIVE_GAS)))
    values = {"GasRegenCmd": self.INACTIVE_GAS, "GasRegenCmdActive": 1}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)))

  def test_lateral_brake_pauses_steering_and_disengages_longitudinal(self):
    self._enable_lateral()
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(10))
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self._tx(self._send_brake_msg(1)))
    self.assertFalse(self._tx(self._send_gas_msg(1)))
    self.assertFalse(self.safety.safety_a.state.lateral_suspended)
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lateral_cancel_latches_through_readiness_recovery(self):
    self._enable_lateral()
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self.safety.safety_a.state.lateral_suspended)
    self.safety.safety_set_lateral_request(False, True)
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.safety.safety_set_lateral_request(False, False)
    self.safety.safety_set_lateral_request(True, True)
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lateral_cancel_rearms_on_real_set_resume(self):
    for button in (Buttons.DECEL_SET, Buttons.RES_ACCEL):
      with self.subTest(button=button):
        self._reset_safety_hooks()
        self._enable_lateral()
        self._rx(self._button_msg(Buttons.CANCEL))
        self._rx(self._button_msg(button))
        self._rx(self._button_msg(Buttons.UNPRESS))
        # Test lateral independently of the normal cruise permission these buttons grant.
        self.safety.set_controls_allowed(False)
        self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_request_expiry_latches(self):
    self._enable_lateral()
    self.safety.set_timer(500000)
    self.safety.set_rx_checks_received(-1, 0)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self.safety.safety_a.state.lateral_suspended)

  def test_lateral_lease_boundary_and_timer_wrap(self):
    for start in (1000000, 0xFFFFFF00):
      with self.subTest(start=start):
        self._reset_safety_hooks()
        self.safety.set_timer(start)
        self._enable_lateral()
        self.safety.set_timer((start + 499999) & 0xFFFFFFFF)
        self.assertTrue(self._tx(self._torque_cmd_msg(1)))
        self.safety.set_timer((start + 500000) & 0xFFFFFFFF)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_safety_tick_revokes_stale_permission(self):
    self._enable_lateral()
    self.safety.set_timer(500000)
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_a.state.lateral_allowed)
    self.assertTrue(self.safety.safety_a.state.lateral_suspended)

  def test_lateral_old_lease_cannot_be_hidden_by_heartbeat(self):
    self._enable_lateral()
    self.safety.set_timer(500000)
    self.safety.set_rx_checks_received(-1, 0)
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self.safety.safety_a.state.lateral_suspended)

  def test_lateral_every_rx_must_be_recent(self):
    for index in range(self.safety.get_rx_checks_len()):
      with self.subTest(index=index):
        self._reset_safety_hooks()
        self._enable_lateral()
        self.safety.set_timer(500000)
        self.safety.set_rx_checks_received(index, 500000)
        self.safety.safety_set_lateral_request(True, True)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))
        self.assertTrue(self.safety.safety_a.state.lateral_suspended)

  def test_lateral_fault_during_benign_pause_stays_latched(self):
    for pause in ('brake', 'reverse', 'not_ready'):
      for fault in ('heartbeat', 'can', 'invalid_gear'):
        with self.subTest(pause=pause, fault=fault):
          self._reset_safety_hooks()
          self.safety.set_timer(0)
          self._enable_lateral()
          if pause == 'brake':
            self._rx(self._user_brake_msg(True))
          elif pause == 'reverse':
            self._rx(self._gear_msg(2))
          else:
            self.safety.safety_set_lateral_request(False, True)
          self.assertFalse(self.safety.safety_a.state.lateral_allowed)
          self.assertTrue(self.safety.safety_a.state.lateral_armed)
          self.assertFalse(self.safety.safety_a.state.lateral_suspended)
          if fault == 'heartbeat':
            self.safety.set_timer(500000)
            self.safety.set_rx_checks_received(-1, 0)
            self.safety.safety_tick()
          elif fault == 'can':
            self.safety.set_timer(400000)
            self.safety.safety_set_lateral_request(False, True)
            self.safety.set_timer(500000)
            self.safety.safety_tick()
          else:
            self._rx(self._gear_msg(15))
          self.assertTrue(self.safety.safety_a.state.lateral_suspended)
          self._fresh_inputs()
          self.safety.safety_set_lateral_request(True, True)
          self.assertFalse(self._tx(self._torque_cmd_msg(1)))
          self.safety.safety_set_lateral_request(False, False)
          self.safety.safety_set_lateral_request(True, True)
          self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_missing_inputs_before_first_arm_do_not_latch(self):
    self.safety.safety_set_lateral_request(True, True)
    self.safety.set_timer(600000)
    self.safety.safety_tick()
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self.safety.safety_a.state.lateral_armed)
    self.assertFalse(self.safety.safety_a.state.lateral_suspended)
    self._fresh_inputs()
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_unknown_rx_cannot_boot(self):
    self._rx(self._gear_msg(4))
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_wrong_bus_gear_cannot_boot(self):
    self._fresh_inputs(1)
    self._rx(self.packer.make_can_msg_safety("ECMPRDNL2", 2, {"PRNDL2": 4}))
    self.safety.safety_set_lateral_request(True, True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_reverse_pauses_and_drive_resumes(self):
    self._enable_lateral()
    self._rx(self._gear_msg(2))
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self.safety.safety_a.state.lateral_suspended)
    self._rx(self._gear_msg(4))
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_invalid_gear_latches(self):
    self._enable_lateral()
    self._rx(self._gear_msg(15))
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self.safety.safety_a.state.lateral_suspended)
    self._rx(self._gear_msg(4))
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_relay_fault_stops(self):
    self._enable_lateral()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_torque_rate_driver_and_request_limits(self):
    self._enable_lateral()
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP + 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(1, steer_req=0)))
    self.safety.set_desired_torque_last(300)
    self.safety.set_rt_torque_last(300)
    self.assertFalse(self._tx(self._torque_cmd_msg(301)))
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    self.safety.set_torque_driver(-400, -400)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_only_supported_silverado_camera_long(self):
    for flags in (0, GMSafetyFlags.HW_CAM, GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG,
                  GMSafetyFlags.SILVERADO_TORQUE, GMSafetyFlags.HW_CAM | GMSafetyFlags.SILVERADO_TORQUE,
                  GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.SILVERADO_TORQUE | GMSafetyFlags.EV):
      with self.subTest(flags=flags):
        self.safety.set_safety_hooks(self.SAFETY_MODEL, flags | GMSafetyFlags.LATERAL_ONLY)
        self.safety.safety_set_lateral_request(True, True)
        self.assertFalse(self.safety.safety_a.setup.gm_lateral_only)
        self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_lateral_capability_clears_when_safety_mode_changes(self):
    for mode in (CarParams.SafetyModel.silent, CarParams.SafetyModel.noOutput,
                 CarParams.SafetyModel.hondaNidec, CarParams.SafetyModel.toyota):
      with self.subTest(mode=mode):
        self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM)
        self._enable_lateral()
        self.safety.set_safety_hooks(mode, 0)
        self.assertFalse(self.safety.safety_a.setup.gm_lateral_only)
        self.safety.safety_set_lateral_request_flags(3)
        self.assertFalse(self.safety.safety_a.state.lateral_allowed)
        self.assertFalse(self.safety.safety_a.state.lateral_armed)
        self.assertFalse(self.safety.safety_a.state.lateral_enabled)

  def test_lateral_safety_mode_reset_revokes_permission(self):
    self._enable_lateral()
    self._reset_safety_hooks()
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self.safety.safety_a.state.lateral_enabled)


class TestGmCameraLongitudinalEVSafety(TestGmCameraLongitudinalSafety, TestGmEVSafetyBase):
  pass


class TestGmIgnition(common.SafetyTestBase):
  DBC = "gm_global_a_powertrain_generated"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, mode):
    return self.packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": mode})

  # SystemPowerMode 2=Run, 3=Crank Request
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
