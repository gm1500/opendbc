"""Stock-linear branch regressions. CAN packing is mocked; this is not a road/safety test."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from opendbc.car import structs
from opendbc.car.gm import carcontroller as gm_carcontroller
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, CAMERA_ACC_CAR, SDGM_CAR, CarControllerParams


class Actuators(SimpleNamespace):
  def as_builder(self):
    return copy.copy(self)


class TestStockLinear(unittest.TestCase):
  def make_controller(self, candidate=CAR.CHEVROLET_SILVERADO):
    # Exercise the actual update method without requiring live CAN or a vehicle model.
    # Deliberately omit mass/wheelRadius: longitudinal conversion must not read them.
    CP = SimpleNamespace(carFingerprint=candidate, openpilotLongitudinalControl=True, radarUnavailable=True,
                         networkLocation=gm_carcontroller.NetworkLocation.fwdCamera)
    controller = gm_carcontroller.CarController.__new__(gm_carcontroller.CarController)
    controller.CP = CP
    controller.params = CarControllerParams(CP)
    controller.frame = 0
    controller.start_time = 0.
    controller.apply_torque_last = 0
    controller.apply_gas = 0
    controller.apply_brake = 0
    controller.last_steer_frame = 0
    controller.last_button_frame = 0
    controller.cancel_counter = 0
    controller.lka_steering_cmd_counter = 0
    controller.lka_steering_cmd_counter_initialized = False
    controller.packer_pt = controller.packer_ch = controller.packer_obj = None
    return controller

  def update(self, controller, accel, speed=15., stopping=False, active=True, enabled=True, now_nanos=1_000_000_000):
    state = gm_carcontroller.LongCtrlState.stopping if stopping else gm_carcontroller.LongCtrlState.pid
    CC = SimpleNamespace(actuators=Actuators(accel=accel, torque=0., longControlState=state),
                         hudControl=SimpleNamespace(visualAlert=None, setSpeed=20.),
                         latActive=False, longActive=active, enabled=enabled)
    CS = SimpleNamespace(out=SimpleNamespace(vEgo=speed, standstill=speed == 0., steeringTorque=0.),
                         cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0,
                         loopback_lka_steering_cmd_ts_nanos=0, pscm_status={})
    with patch.object(gm_carcontroller, 'gmcan') as can:
      output, _ = controller.update(CC, CS, now_nanos)
      return output, can

  def test_upstream_gas_limits(self):
    for candidate in CAMERA_ACC_CAR | SDGM_CAR:
      params = CarControllerParams(SimpleNamespace(carFingerprint=candidate))
      self.assertEqual(params.MAX_GAS, 1346.)
      self.assertEqual(params.MAX_ACC_REGEN, -540.)
      self.assertEqual(params.INACTIVE_REGEN, -500.)
      self.assertEqual(params.GAS_LOOKUP_BP, [0., 0., 2.])
      self.assertEqual(params.GAS_LOOKUP_V, [-540., 0., 1346.])
    for candidate, threshold in ((CAR.CHEVROLET_VOLT, -1.), (CAR.GMC_ACADIA, -0.1)):
      params = CarControllerParams(SimpleNamespace(carFingerprint=candidate))
      self.assertEqual(params.MAX_GAS, 1018.)
      self.assertEqual(params.GAS_LOOKUP_BP, [threshold, 0., 2.])

  def test_linear_positive_commands_are_speed_independent(self):
    for speed in (0., 1., 2., 4., 5., 10., 20., 35.):
      for accel in (0., 0.1, 0.25, 0.5, 1., 1.5, 2., 3.):
        with self.subTest(speed=speed, accel=accel):
          output, _ = self.update(self.make_controller(), accel, speed)
          self.assertAlmostEqual(output.gas, 673. * min(accel, 2.))
          self.assertEqual(output.brake, 0)

  def test_negative_commands_and_limits(self):
    for speed in (0., 2., 3., 4., 5., 20., 35.):
      for accel in np.linspace(-6., -0.02, 101):
        output, _ = self.update(self.make_controller(), accel, speed)
        self.assertEqual(output.gas, -540.)
        self.assertGreaterEqual(output.brake, 0)
        self.assertLessEqual(output.brake, 400)
    output, _ = self.update(self.make_controller(), -8.)
    self.assertEqual(output.brake, 400)

  def test_inactive_never_brakes_or_propels(self):
    for accel in (-4., 0., 2.):
      for stopping in (False, True):
        for enabled in (False, True):
          output, can = self.update(self.make_controller(), accel, 0., stopping, False, enabled)
          self.assertEqual((output.gas, output.brake), (-500., 0))
          self.assertEqual(can.create_gas_regen_command.call_args.args[4], enabled)
          self.assertFalse(can.create_gas_regen_command.call_args.args[5])

  def test_stopping_override_and_stop_flags(self):
    for accel in (-0.37, 0., 2.):
      output, can = self.update(self.make_controller(), accel, 0., stopping=True)
      self.assertEqual(output.gas, -540.)
      self.assertTrue(can.create_gas_regen_command.call_args.args[5])
      self.assertTrue(can.create_friction_brake_command.call_args.args[5])
      self.assertTrue(can.create_friction_brake_command.call_args.args[6])
    _, can = self.update(self.make_controller(), 0., 0., stopping=False)
    self.assertFalse(can.create_gas_regen_command.call_args.args[5])

  def test_low_speed_brake_calibration(self):
    for speed, expected in ((0., 49), (2., 49), (3., 43), (4., 37), (20., 37)):
      output, _ = self.update(self.make_controller(), -0.37, speed)
      self.assertEqual(output.brake, expected)
    for accel, expected in ((-0.5, 62), (-0.2, 32), (-0.1, 10), (-1., 100), (-4., 400)):
      output, _ = self.update(self.make_controller(), accel, 1.)
      self.assertEqual(output.brake, expected)

  def test_tiny_brake_guard_only_blocks_initial_one_unit(self):
    output, _ = self.update(self.make_controller(), -0.01, 10.)
    self.assertEqual(output.brake, 0)
    controller = self.make_controller()
    controller.apply_brake = 2
    output, _ = self.update(controller, -0.01, 10.)
    self.assertEqual(output.brake, 1)
    output, _ = self.update(self.make_controller(), -0.02, 10.)
    self.assertEqual(output.brake, 2)
    output, _ = self.update(self.make_controller(), -0.01, 4.)
    self.assertEqual(output.brake, 1)
    output, _ = self.update(self.make_controller(), -0.01, 10., stopping=True)
    self.assertEqual(output.brake, 1)

  def test_brake_release_has_no_road_load_gas(self):
    controller = self.make_controller()
    output, _ = self.update(controller, -0.5, 20.)
    self.assertGreater(output.brake, 0)
    controller.frame = 4
    output, _ = self.update(controller, 0., 20.)
    self.assertEqual((output.gas, output.brake), (0., 0))

  def test_other_gm_stopping_remains_upstream(self):
    for candidate, expected in ((CAR.CHEVROLET_BOLT_EUV, -500.), (CAR.CHEVROLET_VOLT, -650.)):
      output, _ = self.update(self.make_controller(candidate), 1., 0., stopping=True)
      self.assertEqual(output.gas, expected)

  def test_steering_counter_initializes_once_and_wraps(self):
    controller = self.make_controller()
    for frame, expected in ((4, 1), (8, 2), (12, 3), (16, 0), (20, 1)):
      controller.frame = frame
      _, can = self.update(controller, 0.)
      self.assertEqual(can.create_steering_control.call_args.args[3], expected)
      self.assertTrue(controller.lka_steering_cmd_counter_initialized)
    controller.frame = 24
    _, can = self.update(controller, 0., now_nanos=10_000_000)
    can.create_steering_control.assert_not_called()
    self.assertEqual(controller.lka_steering_cmd_counter, 2)

  def test_upstream_ki_and_retained_sierra_settings(self):
    ret = structs.CarParams.new_message()
    ret = CarInterface._get_params(ret, CAR.CHEVROLET_SILVERADO, {0: {}, 1: {}, 2: {}}, [], True, False, False)
    self.assertEqual(list(ret.longitudinalTuning.kiBP), [5., 35.])
    self.assertEqual(list(ret.longitudinalTuning.kiV), [2., 1.5])
    self.assertAlmostEqual(ret.stopAccel, -0.37, places=6)
    self.assertAlmostEqual(ret.longitudinalActuatorDelay, 0.5)
    self.assertAlmostEqual(ret.steerActuatorDelay, 0.27, places=6)
    self.assertEqual(ret.minSteerSpeed, -1.)
    self.assertTrue(ret.openpilotLongitudinalControl)

  def test_torque_converter_is_removed(self):
    self.assertFalse(hasattr(gm_carcontroller.CarController, 'accel_to_torque'))
    params = self.make_controller().params
    for name in ('MAX_TORQUE', 'MIN_TORQUE', 'INACTIVE_TORQUE', 'DRAG_FORCE_FACTOR', 'ROLLING_RESISTANCE_COEFFICIENT'):
      self.assertFalse(hasattr(params, name))


if __name__ == '__main__':
  unittest.main()
