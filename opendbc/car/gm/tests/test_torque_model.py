import unittest
from types import SimpleNamespace

import numpy as np

from opendbc.car import gen_empty_fingerprint
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests import test_brake_map
from opendbc.car.gm.values import CAR, DBC, CarControllerParams


class TestTorqueModel(unittest.TestCase):
  @staticmethod
  def controller(candidate=CAR.CHEVROLET_SILVERADO):
    cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], True, False, False)
    return CarController(DBC[candidate], cp)

  @staticmethod
  def torque(controller, speed, accel=0.):
    return controller.accel_to_torque(accel, SimpleNamespace(out=SimpleNamespace(vEgo=speed)), 0.)

  def test_sierra_road_load_across_speeds(self):
    controller = self.controller()
    self.assertAlmostEqual(controller.CP.mass, 2586.)
    self.assertAlmostEqual(controller.CP.wheelRadius, .419)
    # Independent force balance: Cd=.30, area=3.61 m^2, rho=1.225, effective Crr=.004, road-load scale=.75.
    for kph in [10., 30., 60., 90., 120.]:
      speed = kph / 3.6
      expected = .419 * .75 * (.5 * 1.225 * .30 * 3.61 * speed**2 + .004 * 2586. * 9.81)
      with self.subTest(kph=kph):
        self.assertAlmostEqual(self.torque(controller, speed), expected, places=4)

  def test_fingerprint_selects_only_sierra_road_load(self):
    for candidate in CAR:
      controller = self.controller(candidate)
      p = controller.params
      with self.subTest(candidate=candidate):
        self.assertEqual(p.STOPPING_DRAG_FORCE_FACTOR, .3)
        if candidate == CAR.CHEVROLET_SILVERADO:
          self.assertAlmostEqual(p.DRAG_FORCE_FACTOR, .6633375)
          self.assertEqual(p.ROLLING_RESISTANCE_COEFFICIENT, .004)
          self.assertEqual(p.ROAD_LOAD_SCALE, .75)
        else:
          self.assertEqual(p.DRAG_FORCE_FACTOR, .3)
          self.assertEqual(p.ROLLING_RESISTANCE_COEFFICIENT, 0.)
          self.assertEqual(p.ROAD_LOAD_SCALE, 1.)
          for speed in [0., .2, 1., 15., 35.]:
            for accel in [-4., -.1, 0., .5, 2.]:
              expected = controller.CP.wheelRadius * (controller.CP.mass * accel + .3 * speed**2)
              self.assertEqual(self.torque(controller, speed, accel), expected)

  def test_acceleration_gain_and_smooth_speed_dependence(self):
    controller = self.controller()
    for speed in [0., .5, 1., 10., 20., 40.]:
      gain = self.torque(controller, speed, .1) - self.torque(controller, speed)
      self.assertAlmostEqual(gain, .1 * controller.CP.mass * controller.CP.wheelRadius)
    speeds = np.linspace(0., 40., 1001)
    torques = [self.torque(controller, speed) for speed in speeds]
    self.assertTrue(np.all(np.diff(torques) >= 0.))
    for speed in [0., 1.]:
      self.assertLess(abs(self.torque(controller, speed + 1e-6) - self.torque(controller, speed - 1e-6)), .001)

  def test_no_extra_launch_torque_at_rest(self):
    controller = self.controller()
    self.assertEqual(self.torque(controller, 0.), 0.)
    self.assertAlmostEqual(self.torque(controller, 0., .5), .5 * controller.CP.mass * controller.CP.wheelRadius)

  def test_propulsion_encoding_and_limits(self):
    for speed in [0., .5, 10., 20., 40.]:
      controller = self.controller()
      for accel in [0., .1, 2., 4.]:
        output, messages = test_brake_map.TestBrakeMap.command(speed, accel)
        expected = round(np.clip(self.torque(controller, speed, min(accel, 2.)), -540., 2450.))
        with self.subTest(speed=speed, accel=accel):
          self.assertEqual(output.gas, expected)
          self.assertEqual(output.brake, 0.)
          data = messages[715]
          decoded = (((data[1] & 7) << 16) | (data[2] << 8) | data[3]) * .125 - 22534.
          self.assertEqual(decoded, expected)

  def test_shared_force_balance_avoids_conflicting_gas_and_brake_requests(self):
    controller = self.controller()
    self.assertGreater(self.torque(controller, 60. / 3.6, -.08), 0.)
    output, _ = test_brake_map.TestBrakeMap.command(60. / 3.6, -.08)
    self.assertEqual(output.brake, 0.)
    self.assertGreater(output.gas, 0.)
    output, _ = test_brake_map.TestBrakeMap.command(60. / 3.6, -.3)
    self.assertGreater(output.brake, 0.)
    self.assertEqual(output.gas, -540.)

  def test_final_stop_calibration_does_not_depend_on_new_road_load(self):
    cp = self.controller().CP
    before = CarControllerParams(cp)
    for speed in [1., 5., 60. / 3.6, 35.]:
      for accel in [-.1, -.5, -1., -4.]:
        output, _ = test_brake_map.TestBrakeMap.command(speed, accel, stopping=True)
        request = min(accel + .3 * speed**2 / cp.mass, 0.)
        normal = np.interp(request, before.BRAKE_LOOKUP_BP, before.BRAKE_LOOKUP_V)
        slow = np.interp(request, before.BRAKE_LOOKUP_BP, before.BRAKE_LOOKUP_V_LOW_SPEED)
        expected = round(np.interp(speed, [2., 4.], [slow, normal]))
        self.assertEqual(output.brake, expected)


if __name__ == '__main__':
  unittest.main()
