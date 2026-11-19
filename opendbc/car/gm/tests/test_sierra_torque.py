"""Compatibility and actuator behavior for the proven Sierra/Silverado tune."""
import unittest

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.values import PLATFORMS

LongCtrlState = structs.CarControl.Actuators.LongControlState


def interface(candidate=CAR.CHEVROLET_SILVERADO, alpha_long=True):
  cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long, False, False)
  ci = CarInterface(cp)
  ci.update([])
  return ci


def command(ci, speed, accel, stopping=False, active=True):
  ci.CS.out = structs.CarState(vEgo=float(speed), standstill=speed == 0.).as_reader()
  cc = structs.CarControl(enabled=active, longActive=active)
  cc.actuators.accel = float(accel)
  cc.actuators.longControlState = LongCtrlState.stopping if stopping else LongCtrlState.pid
  # Isolate a single transmitted longitudinal sample from message decimation.
  ci.CC.frame = 0
  return ci.apply(cc.as_reader(), 1_000_000_000)[0]


class TestSierraTorque(unittest.TestCase):
  def test_required_calibration_is_local_to_silverado(self):
    ci = interface()
    self.assertEqual(ci.CC.torque_mass, 2586.)
    self.assertAlmostEqual(ci.CC.torque_wheel_radius, .419)
    self.assertAlmostEqual(ci.CP.centerToFront / ci.CP.wheelbase, .75)
    self.assertAlmostEqual(ci.CP.stopAccel, -.37)
    self.assertAlmostEqual(ci.CP.longitudinalActuatorDelay, .5)
    self.assertEqual(ci.CC.params.MAX_TORQUE, 2450.)
    self.assertTrue(ci.CP.safetyConfigs[0].safetyParam & GMSafetyFlags.SILVERADO_TORQUE)
    self.assertFalse(ci.CP.autoResumeSng)
    # Do not reintroduce generic mass fields removed by upstream.
    self.assertNotIn('mass', CarParams.schema.fields)

  def test_all_gm_platforms_initialize_without_optional_mass(self):
    for candidate in CAR:
      for alpha_long in (False, True):
        with self.subTest(candidate=candidate, alpha_long=alpha_long):
          ci = interface(candidate, alpha_long)
          command(ci, 12., 0., active=False)
          if candidate != CAR.CHEVROLET_SILVERADO:
            self.assertIsNone(candidate.config.specs.mass)
            self.assertIsNone(candidate.config.specs.wheelRadius)
            self.assertIsNone(candidate.config.specs.dragArea)
            self.assertFalse(hasattr(ci.CC, 'torque_mass'))
            self.assertFalse(ci.CP.safetyConfigs[0].safetyParam & GMSafetyFlags.SILVERADO_TORQUE)

  def test_other_brands_still_use_upstream_parameter_contract(self):
    for platform, platform_enum in PLATFORMS.items():
      if platform_enum in CAR:
        continue
      with self.subTest(platform=platform):
        cp = interfaces[platform].get_non_essential_params(platform)
        self.assertGreater(cp.wheelbase, 0.)
        self.assertGreater(cp.rotationalInertiaPerMass, 0.)
        self.assertNotIn('mass', cp.schema.fields)

  def test_minus_100_nm_coast_and_friction_handoff(self):
    ci = interface()
    speed = 40. / 3.6
    ci.CS.out = structs.CarState(vEgo=speed).as_reader()
    road_load = ci.CC.accel_to_torque(0., ci.CS)
    accel_for_coast = (-100. - road_load) / (ci.CC.torque_mass * ci.CC.torque_wheel_radius)
    coast = command(ci, speed, accel_for_coast)
    self.assertEqual(coast.gas, -100.)
    self.assertEqual(coast.brake, 0.)
    braking = command(ci, speed, -1.)
    self.assertGreater(braking.brake, 0.)
    self.assertEqual(braking.gas, ci.CC.params.BRAKE_TORQUE)

  def test_stopping_and_inactive_commands_remain_bounded(self):
    ci = interface()
    stopping = command(ci, 0., ci.CP.stopAccel, stopping=True)
    self.assertGreater(stopping.brake, 0.)
    self.assertLessEqual(stopping.brake, ci.CC.params.MAX_BRAKE)
    self.assertEqual(stopping.gas, -540.)
    inactive = command(ci, 20., 2., active=False)
    self.assertEqual(inactive.gas, -500.)
    self.assertEqual(inactive.brake, 0.)
    ci.CS.out = structs.CarState(vEgo=0.).as_reader()
    self.assertEqual(ci.CC.accel_to_torque(0., ci.CS), 0.)

  def test_other_gm_actuator_mapping_remains_stock(self):
    for candidate in (CAR.CHEVROLET_BOLT_EUV, CAR.GMC_ACADIA):
      ci = interface(candidate)
      for speed in (0., 2., 20.):
        for accel in (-3., -.5, 0., 1.):
          with self.subTest(candidate=candidate, speed=speed, accel=accel):
            out = command(ci, speed, accel)
            expected_gas = np.interp(accel, ci.CC.params.GAS_LOOKUP_BP, ci.CC.params.GAS_LOOKUP_V)
            expected_brake = round(np.interp(accel, ci.CC.params.BRAKE_LOOKUP_BP, ci.CC.params.BRAKE_LOOKUP_V))
            self.assertAlmostEqual(out.gas, expected_gas, places=3)
            self.assertEqual(out.brake, expected_brake)
