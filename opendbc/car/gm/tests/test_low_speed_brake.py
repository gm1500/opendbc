import unittest
from unittest.mock import patch

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.gm.carcontroller import CarController, compensate_silverado_brake
from opendbc.car.gm.carstate import CarState
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, DBC


class TestLowSpeedBrake(unittest.TestCase):
  def test_bounds_and_monotonicity(self):
    requests = np.linspace(-4., 0., 801)
    for speed in np.linspace(-1., 40., 165):
      corrected = np.array([compensate_silverado_brake(a, speed) for a in requests])
      extra = requests - corrected
      self.assertTrue(np.all(extra >= -1e-12))
      self.assertTrue(np.all(extra <= 0.10 + 1e-12))
      self.assertTrue(np.all(extra <= -requests * 0.25 + 1e-12))
      self.assertTrue(np.all(np.diff(corrected) >= 0.))
      if speed <= 0. or speed >= 2.:
        np.testing.assert_array_equal(corrected, requests)

  def test_no_braking_from_nonnegative_request(self):
    for speed in [0., 0.2, 0.5, 1., 1.5, 2., 20.]:
      for accel in [0., 0.01, 0.5, 2.]:
        self.assertEqual(compensate_silverado_brake(accel, speed), accel)

  def test_continuous_speed_and_request_boundaries(self):
    for speed in [0., 0.3, 1., 2.]:
      self.assertLess(abs(compensate_silverado_brake(-0.5, speed - 1e-6) -
                          compensate_silverado_brake(-0.5, speed + 1e-6)), 1e-6)
    for accel in [-0.4, 0.]:
      self.assertLess(abs(compensate_silverado_brake(accel - 1e-6, 0.5) -
                          compensate_silverado_brake(accel + 1e-6, 0.5)), 3e-6)

  @staticmethod
  def command(candidate, speed, accel, active=True, standstill=False, stopping=False, compensate=True):
    cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], True, False, False)
    controller = CarController(DBC[candidate], cp)
    controller.frame = 4  # longitudinal output, without the unrelated 10-frame PSCM forwarding
    cs = CarState(cp)
    cs.out = structs.CarState(vEgo=speed, standstill=standstill)
    cc = structs.CarControl(enabled=active, longActive=active)
    cc.actuators.accel = accel
    cc.actuators.longControlState = 'stopping' if stopping else 'pid'
    cc = cc.as_reader()
    if compensate:
      return controller.update(cc, cs, 1_000_000_000)
    with patch('opendbc.car.gm.carcontroller.compensate_silverado_brake', side_effect=lambda a, v: a):
      return controller.update(cc, cs, 1_000_000_000)

  def test_controller_scope_and_hold(self):
    cases = [
      (CAR.CHEVROLET_SILVERADO, 2., -0.3, True, False, False),
      (CAR.CHEVROLET_SILVERADO, 60 / 3.6, -0.3, True, False, False),
      (CAR.CHEVROLET_SILVERADO, 0.5, 0.3, True, False, False),
      (CAR.CHEVROLET_SILVERADO, 0.5, -0.3, False, False, False),
      (CAR.CHEVROLET_SILVERADO, 0., -0.37, True, True, True),
      (CAR.CHEVROLET_SILVERADO, 0.15, -0.37, True, True, True),
      (CAR.CHEVROLET_EQUINOX, 0.5, -0.3, True, False, False),
      (CAR.CHEVROLET_BOLT_EUV, 0.5, -0.3, True, False, False),
    ]
    for case in cases:
      with self.subTest(case=case):
        original, original_can = self.command(*case, compensate=False)
        tuned, tuned_can = self.command(*case)
        self.assertEqual(original.to_dict(), tuned.to_dict())
        self.assertEqual(original_can, tuned_can)

  def test_controller_braking_and_limit(self):
    for accel in [-0.2, -0.4, -1., -4.]:
      original, _ = self.command(CAR.CHEVROLET_SILVERADO, 0.5, accel, compensate=False)
      tuned, _ = self.command(CAR.CHEVROLET_SILVERADO, 0.5, accel)
      self.assertEqual(tuned.gas, -500.)
      self.assertGreaterEqual(tuned.brake, original.brake)
      self.assertLessEqual(tuned.brake - original.brake, 10.)
      self.assertLessEqual(tuned.brake, 400.)
      if accel > -4.:
        self.assertGreater(tuned.brake, original.brake)


if __name__ == '__main__':
  unittest.main()
