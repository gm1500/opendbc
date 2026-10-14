import unittest

from opendbc.car.gm import gmcan
from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests import test_gm

Buttons = test_gm.Buttons


class TestGmResumeSetSafety(test_gm.TestGmCameraLongitudinalSafety):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.RESUME_SET_TEST
  TX_MSGS = [*test_gm.TestGmCameraLongitudinalSafety.TX_MSGS, [0x1E1, 0]]

  def snapshot(self, speed=0., cruise=4, counter=0, now=1_000_000):
    self.safety.set_timer(now)
    self._rx(self._speed_msg(speed))  # DBC units are km/h
    self._rx(self.packer.make_can_msg_safety('AcceleratorPedal2', 0, {'CruiseState': cruise}))
    self._rx(self.button(Buttons.UNPRESS, counter))
    self.safety.set_controls_allowed(True)

  def button(self, button, counter=1, bus=0):
    addr, data, _ = gmcan.create_buttons(self.packer, bus, counter, button)
    return libsafety_py.make_CANPacket(addr, bus, data)

  def test_press_requires_controls_and_factory_standstill(self):
    for button in (Buttons.RES_ACCEL, Buttons.DECEL_SET):
      for controls in (False, True):
        for cruise in range(8):
          with self.subTest(button=button, controls=controls, cruise=cruise):
            self.setUp()
            self.snapshot(cruise=cruise)
            self.safety.set_controls_allowed(controls)
            self.assertEqual(self._tx(self.button(button)), controls and cruise == 4)

  def test_speed_boundary_on_either_rear_wheel(self):
    for speed in (0., 6., 7.1, 7.2, 20.):
      for wheel in ('RLWheelSpd', 'RRWheelSpd'):
        self.setUp()
        self.snapshot()
        self._rx(self.packer.make_can_msg_safety('EBCMWheelSpdRear', 0, {wheel: speed}))
        self.assertEqual(self._tx(self.button(Buttons.RES_ACCEL)), speed < 7.2)

  def test_fresh_inputs_counter_rate_and_checksum(self):
    self.snapshot()
    self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL, 0)))
    self.assertTrue(self._tx(self.button(Buttons.RES_ACCEL, 1)))
    self.safety.set_timer(1_010_000)
    self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL, 1)))
    self.safety.set_timer(1_030_000)
    self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL, 1)))  # one press per observed neutral frame
    self.snapshot(counter=1, now=1_040_000)
    msg = self.button(Buttons.RES_ACCEL, 2)
    msg[0].data[6] ^= 1
    self.assertFalse(self._tx(msg))
    self.assertTrue(self._tx(self.button(Buttons.RES_ACCEL, 2)))
    self.safety.set_timer(1_150_001)
    self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL, 2)))

  def test_no_press_before_all_inputs_seen(self):
    for missing in ('speed', 'cruise', 'button'):
      self.setUp()
      self.safety.set_timer(1_000_000)
      if missing != 'speed':
        self._rx(self._speed_msg(0.))
      if missing != 'cruise':
        self._rx(self.packer.make_can_msg_safety('AcceleratorPedal2', 0, {'CruiseState': 4}))
      if missing != 'button':
        self._rx(self.button(Buttons.UNPRESS, 0))
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL)))

  def test_driver_pedals_and_non_neutral_native_button_veto(self):
    for msg in (self._user_brake_msg(True), self._user_gas_msg(True), self.button(Buttons.DECEL_SET, 0)):
      self.setUp()
      self.snapshot()
      self._rx(msg)
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL)))

  def test_only_three_button_values_exact_shape_and_bus(self):
    for button in range(1, 8):
      self.setUp()
      self.snapshot()
      self.assertEqual(self._tx(self.button(button)), button in (1, 2, 3))
    for offset, bit in ((0, 1), (1, 1), (2, 0x40), (3, 2), (4, 0x80), (5, 0x80)):
      self.setUp()
      self.snapshot()
      msg = self.button(Buttons.RES_ACCEL)
      msg[0].data[offset] ^= bit
      self.assertFalse(self._tx(msg))
    for bus in (1, 2):
      self.setUp()
      self.snapshot()
      self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL, bus=bus)))

  def test_unpress_after_abort_and_echo_cannot_reengage(self):
    for button in (Buttons.RES_ACCEL, Buttons.DECEL_SET):
      self.setUp()
      self.snapshot()
      self.assertTrue(self._tx(self.button(button)))
      self.safety.set_controls_allowed(False)
      self.safety.set_timer(1_040_000)
      self._rx(self.button(button))
      self._rx(self.button(Buttons.UNPRESS, 2))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertFalse(self._tx(self.button(button, 3)))
      self.assertTrue(self._tx(self.button(Buttons.UNPRESS, 3)))

  def test_other_safety_configurations_do_not_gain_button_permission(self):
    for flags in (0, 1, 2, 3, 4, 7, 8, 9, 10, 12, 15):
      self.safety.set_safety_hooks(CarParams.SafetyModel.gm, flags)
      self.snapshot()
      self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL)), flags)
      self.assertFalse(self._tx(self.button(Buttons.DECEL_SET)), flags)

  def test_release_build_does_not_allow_diagnostic(self):
    # Loading the production library through the existing helper preserves CAN
    # struct ABI; reset the debug library afterwards for the rest of this suite.
    debug = libsafety_py.libsafety
    try:
      libsafety_py.load(libsafety_py._build_libsafety(release=True))
      self.safety = libsafety_py.libsafety
      self.safety.set_safety_hooks(CarParams.SafetyModel.gm, 11)
      self.snapshot()
      self.assertFalse(self._tx(self.button(Buttons.RES_ACCEL)))
    finally:
      libsafety_py.libsafety = debug


if __name__ == '__main__':
  unittest.main()
