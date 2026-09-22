import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from opendbc.car import structs
from opendbc.car.gm.carcontroller import CarController, LongCtrlState
from opendbc.car.gm.values import CAR, DBC


class TestLongitudinalTorque(unittest.TestCase):
  def setUp(self):
    self.cp = structs.CarParams.new_message()
    self.cp.carFingerprint = CAR.CHEVROLET_SILVERADO
    self.cp.mass = 2586.0
    self.cp.wheelRadius = 0.425
    self.cp.openpilotLongitudinalControl = True
    self.cp.networkLocation = structs.CarParams.NetworkLocation.fwdCamera
    self.cp.radarUnavailable = True
    self.cc = structs.CarControl.new_message()
    self.cc.enabled = True
    self.cc.longActive = True
    self.cc.actuators.longControlState = LongCtrlState.pid
    self.cs = SimpleNamespace(out=structs.CarState.new_message(), cam_lka_steering_cmd_counter=0,
                              loopback_lka_steering_cmd_ts_nanos=0, pscm_status={})
    self.cs.out.vEgo = 30.0

  def controller(self):
    with patch('opendbc.car.gm.carcontroller.CANPacker'):
      return CarController(DBC[self.cp.carFingerprint], self.cp)

  def commands(self, controller, accel, stopping=False):
    controller.frame = 0
    self.cc.actuators.accel = accel
    self.cc.actuators.longControlState = LongCtrlState.stopping if stopping else LongCtrlState.pid
    with patch('opendbc.car.gm.carcontroller.gmcan'):
      return controller.update(self.cc.as_reader(), self.cs, 0)[0]

  def baseline(self, controller, accel, stopping=False):
    p = controller.params
    accel = np.clip(accel, p.ACCEL_MIN, p.ACCEL_MAX)
    torque = controller.accel_to_torque(accel, self.cs, 0)
    brake_accel = min((torque - p.BRAKE_THRESHOLD) / (self.cp.mass * self.cp.wheelRadius), 0)
    brake = int(round(np.interp(brake_accel, p.BRAKE_LOOKUP_BP, p.BRAKE_LOOKUP_V)))
    gas = p.INACTIVE_TORQUE if brake > 0 or stopping else int(round(np.clip(torque, p.MIN_TORQUE, p.MAX_TORQUE)))
    return gas, brake

  def test_extra_propulsion_matches_force_units(self):
    controller = self.controller()
    command = self.commands(controller, 0.5)
    expected = self.cp.wheelRadius * (self.cp.mass * 0.5 + 0.5 * self.cs.out.vEgo ** 2)
    self.assertEqual(command.gas, round(expected))
    self.assertEqual(command.brake, 0)

  def test_zero_speed_gets_no_extra_torque(self):
    self.cs.out.vEgo = 0.0
    controller = self.controller()
    command = self.commands(controller, 0.5)
    self.assertEqual((command.gas, command.brake), self.baseline(controller, 0.5))

  def test_braking_coasting_and_hold_unchanged(self):
    controller = self.controller()
    for speed in (0.0, 0.5, 10.0, 30.0, 40.0):
      self.cs.out.vEgo = speed
      for accel in (-6.0, -4.0, -2.5, -0.5, -0.1, -0.02, 0.0):
        for stopping in (False, True):
          with self.subTest(speed=speed, accel=accel, stopping=stopping):
            command = self.commands(controller, accel, stopping)
            self.assertEqual((command.gas, command.brake), self.baseline(controller, accel, stopping))

  def test_blend_is_continuous_and_bounded(self):
    controller = self.controller()
    previous = None
    for accel in np.linspace(-0.001, 0.201, 203):
      command = self.commands(controller, float(accel))
      baseline_gas, baseline_brake = self.baseline(controller, float(accel))
      self.assertEqual(command.brake, baseline_brake)
      self.assertGreaterEqual(command.gas, baseline_gas)
      self.assertLessEqual(command.gas - baseline_gas, 77.0)  # 0.2 * 0.425 * 30^2, rounded
      if previous is not None:
        self.assertLessEqual(command.gas - previous, 3.0)
      previous = command.gas

  def test_existing_torque_and_brake_caps(self):
    controller = self.controller()
    self.cs.out.vEgo = 40.0
    self.assertEqual(self.commands(controller, 10.0).gas, controller.params.MAX_TORQUE)
    self.cs.out.vEgo = 0.0
    self.assertEqual(self.commands(controller, -10.0).brake, controller.params.MAX_BRAKE)

  def test_stopping_and_inactive_never_add_propulsion(self):
    controller = self.controller()
    self.assertEqual(self.commands(controller, 0.5, stopping=True).gas, controller.params.INACTIVE_TORQUE)
    self.cc.longActive = False
    command = self.commands(controller, 0.5)
    self.assertEqual(command.gas, controller.params.INACTIVE_TORQUE)
    self.assertEqual(command.brake, 0)

  def test_other_gm_keeps_baseline(self):
    self.cp.carFingerprint = CAR.CHEVROLET_EQUINOX
    controller = self.controller()
    for accel in (-1.0, 0.0, 0.1, 0.5, 2.0):
      command = self.commands(controller, accel)
      self.assertEqual((command.gas, command.brake), self.baseline(controller, accel))

  def test_stock_long_sends_no_gas_or_brake_commands(self):
    self.cp.openpilotLongitudinalControl = False
    controller = self.controller()
    self.cc.actuators.accel = 0.5
    with patch('opendbc.car.gm.carcontroller.gmcan') as gmcan:
      controller.update(self.cc.as_reader(), self.cs, 0)
      gmcan.create_gas_regen_command.assert_not_called()
      gmcan.create_friction_brake_command.assert_not_called()


if __name__ == '__main__':
  unittest.main()
