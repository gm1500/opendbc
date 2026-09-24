import unittest

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.carstate import CarState
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, DBC, CarControllerParams


class TestBrakeMap(unittest.TestCase):
  @staticmethod
  def params(candidate=CAR.CHEVROLET_SILVERADO, longitudinal=True):
    return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], longitudinal, False, False)

  @classmethod
  def command(cls, speed, accel, *, candidate=CAR.CHEVROLET_SILVERADO, active=True, enabled=True,
              standstill=False, stopping=False, longitudinal=True):
    cp = cls.params(candidate, longitudinal)
    controller = CarController(DBC[candidate], cp)
    controller.frame = 4  # Longitudinal frame without unrelated PSCM forwarding.
    cs = CarState(cp)
    cs.out = structs.CarState(vEgo=speed, standstill=standstill)
    cc = structs.CarControl(enabled=enabled, longActive=active)
    cc.actuators.accel = accel
    cc.actuators.longControlState = 'stopping' if stopping else 'pid'
    output, messages = controller.update(cc.as_reader(), cs, 1_000_000_000)
    return output, {m[0]: m[1] for m in messages}

  def test_brake_commands_and_can_encoding(self):
    for accel, expected in [(-.05, 5), (-.1, 10), (-.2, 32), (-.37, 49), (-.5, 62), (-1., 100), (-4., 400)]:
      with self.subTest(accel=accel):
        output, can = self.command(1., accel)
        self.assertEqual(output.brake, expected)
        self.assertEqual(output.gas, -540.)
        self.assertEqual((-int.from_bytes(can[789][:2], 'big')) & 0xfff, expected)
        self.assertEqual(can[789][0] >> 4, 0xa)
        torque = (((can[715][1] & 7) << 16) | (can[715][2] << 8) | can[715][3]) * .125 - 22534
        self.assertEqual(torque, -540.)

  def test_bounds_monotonicity_and_release(self):
    cp = self.params()
    p = CarControllerParams(cp)
    requests = np.linspace(-5., 2., 1401)
    for speed in [0., .1, 1., 2., 3., 4., 40.]:
      accel = np.clip(requests, p.ACCEL_MIN, p.ACCEL_MAX) + p.DRAG_CONSTANT * speed**2 / cp.mass
      normal = np.interp(accel, p.BRAKE_LOOKUP_BP, p.BRAKE_LOOKUP_V)
      slow = np.interp(accel, p.BRAKE_LOOKUP_BP, p.BRAKE_LOOKUP_V_LOW_SPEED)
      brake = normal + np.interp(speed, [2., 4.], [1., 0.]) * (slow - normal)
      self.assertTrue(np.all((brake >= 0.) & (brake <= 400.)))
      self.assertTrue(np.all(np.diff(brake) <= 1e-10))
      self.assertTrue(np.all(brake[requests >= 0.] == 0.))
      self.assertTrue(np.all(brake - normal <= 12. + 1e-10))
    for speed in [0., 1., 3., 4., 40.]:
      for accel in [0., .01, .5, 2.]:
        output, can = self.command(speed, accel)
        self.assertEqual(output.brake, 0.)
        self.assertEqual(can[789][0] >> 4, 1)

  def test_continuous_speed_and_accel_transitions(self):
    for speed in [0., 2., 4.]:
      before, _ = self.command(speed - 1e-6, -.37)
      after, _ = self.command(speed + 1e-6, -.37)
      self.assertLessEqual(abs(after.brake - before.brake), 1.)
    for accel in [-4., -1., -.5, -.2, -.1, 0.]:
      before, _ = self.command(1., accel - 1e-6)
      after, _ = self.command(1., accel + 1e-6)
      self.assertLessEqual(abs(after.brake - before.brake), 1.)
    self.assertEqual(self.command(3., -.37)[0].brake, 43.)

  def test_high_speed_and_strong_braking_keep_existing_scale(self):
    cp = self.params()
    for speed in [4., 40 / 3.6, 60 / 3.6, 30.]:
      for accel in [-.05, -.2, -.5, -1., -4.]:
        output, _ = self.command(speed, accel)
        expected = round(np.clip(-100 * (accel + .3 * speed**2 / cp.mass), 0, 400))
        self.assertEqual(output.brake, expected)
    for accel in [-1.1, -2., -4., -5.]:
      self.assertEqual(self.command(0., accel)[0].brake, min(round(-100 * accel), 400))

  def test_stop_hold_inactive_and_stock_acc(self):
    moving, _ = self.command(.05, -.37, stopping=True)
    stopped, can = self.command(0., -.37, stopping=True, standstill=True)
    self.assertEqual(moving.brake, stopped.brake)
    self.assertEqual(stopped.brake, 49.)
    self.assertEqual(can[789][0] >> 4, 0xd)
    for enabled in [False, True]:
      output, can = self.command(1., -4., active=False, enabled=enabled)
      self.assertEqual((output.brake, output.gas), (0., -500.))
      self.assertEqual(can[789][0] >> 4, 1)
    _, can = self.command(1., -1., longitudinal=False)
    self.assertNotIn(715, can)
    self.assertNotIn(789, can)

  def test_other_platforms_keep_original_tables(self):
    for candidate in CAR:
      if candidate == CAR.CHEVROLET_SILVERADO:
        continue
      with self.subTest(candidate=candidate):
        p = CarControllerParams(self.params(candidate))
        self.assertEqual(p.BRAKE_LOOKUP_BP, [p.ACCEL_MIN, p.BRAKE_THRESHOLD])
        self.assertEqual(p.BRAKE_LOOKUP_V, [p.MAX_BRAKE, 0.])
        self.assertEqual(p.BRAKE_LOOKUP_V_LOW_SPEED, p.BRAKE_LOOKUP_V)
        self.assertEqual(p.BRAKE_TORQUE, p.INACTIVE_TORQUE)

  def test_longitudinal_timing_and_gains(self):
    cp = self.params()
    self.assertAlmostEqual(cp.longitudinalActuatorDelay, .3)
    np.testing.assert_allclose(cp.longitudinalTuning.kiV, [.05, .05])
    self.assertAlmostEqual(cp.stopAccel, -.37)


if __name__ == '__main__':
  unittest.main()
