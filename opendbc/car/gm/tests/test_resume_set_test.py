import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC as Database
from opendbc.car import Bus
from opendbc.car.gm import gmcan
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.resume_set_test import ResumeSetActuation
from opendbc.car.gm.values import DBC
from opendbc.car.structs import car
from opendbc.safety.tests.libsafety import libsafety_py


class TestResumeSetCan(unittest.TestCase):
  def setUp(self):
    self.cp = car.CarParams.new_message(carFingerprint='CHEVROLET_SILVERADO', openpilotLongitudinalControl=True,
                                       networkLocation='fwdCamera', flags=2, radarUnavailable=True, mass=2586., wheelRadius=.419,
                                       longitudinalActuatorDelay=.5, stopAccel=-.37)
    self.cp.safetyConfigs = [{'safetyModel': 'gm', 'safetyParam': 11}]
    self.cs = car.CarState.new_message(canValid=True, standstill=True, gearShifter='drive', gmCruiseState=4)
    self.cc = car.CarControl.new_message(enabled=True, longActive=True)
    self.cc.actuators.accel, self.cc.actuators.longControlState = -.37, 'stopping'
    self.db = Database(DBC[self.cp.carFingerprint][Bus.pt])
    self.packer = CANPacker(DBC[self.cp.carFingerprint][Bus.pt])
    self.vehicle = NS(out=self.cs, cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0,
                      loopback_lka_steering_cmd_ts_nanos=0,
                      pscm_status=dict.fromkeys(self.db.name_to_msg['PSCMStatus'].sigs, 0),
                      buttons_counter=0, buttons_ts_nanos=0, cruise_buttons=1, resume_set_suppress_until=0)
    self.gm = CarController(DBC[self.cp.carFingerprint], self.cp)
    self.now = 100.
    self.frame = 0
    self.native_counter = 0
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(car.CarParams.SafetyModel.gm, 11)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.messages = []
    for _ in range(5):
      self.step()

  def decode(self, msg):
    addr, data, _ = msg
    values = {}
    for name, sig in self.db.addr_to_msg[addr].sigs.items():
      value = sig.get_raw_value(data)
      if sig.is_signed:
        value -= ((value >> (sig.size - 1)) & 1) * (1 << sig.size)
      values[name] = value * sig.factor + sig.offset
    return values

  def rx(self, msg):
    addr, data, bus = msg
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def step(self, heartbeat=True, native=True, dt=.01):
    self.now += dt
    self.frame += 1
    self.safety.set_timer(int(self.now * 1e6))
    self.rx(self.packer.make_can_msg('EBCMWheelSpdRear', 0, {'RLWheelSpd': abs(self.cs.vEgoRaw) * 3.6,
                                                          'RRWheelSpd': abs(self.cs.vEgoRaw) * 3.6}))
    self.rx(self.packer.make_can_msg('AcceleratorPedal2', 0, {'CruiseState': self.cs.gmCruiseState}))
    if native and self.frame % 3 == 0:
      self.native_counter = (self.native_counter + 1) % 4
      self.vehicle.buttons_counter = self.native_counter
      self.vehicle.buttons_ts_nanos = int(self.now * 1e9)
      self.rx(gmcan.create_buttons(self.packer, 0, self.native_counter, 1))
    if heartbeat:
      self.cc.gmResumeSetTest.heartbeatMonoTime = int(self.now * 1e9)
    output, messages = self.gm.update(self.cc.as_reader(), self.vehicle, int(self.now * 1e9))
    for msg in messages:
      if msg[0] in (0x1E1, 0x2CB, 0x315):
        self.messages.append((self.now, msg, self.decode(msg)))
        addr, data, bus = msg
        self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data)),
                        f'Panda rejected {hex(addr)} {data.hex()} at {self.now}')
    return output, messages

  def press(self, button=2):
    req = self.cc.gmResumeSetTest
    req.requestId = int((self.now + .01) * 1e9)
    req.button, req.active, req.holding = button, True, True
    self.cc.actuators.accel, self.cc.actuators.longControlState = .3, 'pid'
    return self.step()

  def test_native_resume_vectors_match_recording(self):
    expected = ('000000010020ef', '000000010125de', '00000001022acd', '00000001032fbc')
    for counter, data in enumerate(expected):
      self.assertEqual(gmcan.create_buttons(self.packer, 0, counter, 2)[1].hex(), data)

  def test_both_buttons_repeated_five_frame_bursts_then_release_hold(self):
    for button in (2, 3):
      self.setUp()
      self.press(button)
      start = self.now
      for _ in range(150):
        output, _ = self.step()
      buttons = [(t, d) for t, m, d in self.messages if m[0] == 0x1E1]
      sequence = [d['ACCButtons'] for _, d in buttons]
      self.assertEqual(sequence[:14], [button] * 5 + [1] * 2 + [button] * 5 + [1] * 2)
      self.assertGreaterEqual(min(b[0] - a[0] for a, b in zip(buttons, buttons[1:], strict=False)), .025)
      self.assertTrue(output.gmResumeSetActive)
      during = [(m, d) for t, m, d in self.messages if t > start + .05]
      gas = [d for m, d in during if m[0] == 0x2CB]
      brake = [d for m, d in during if m[0] == 0x315]
      self.assertTrue(gas and brake)
      self.assertTrue(all(d['GasRegenFullStopActive'] == 0 and abs(d['GasRegenCmd'] - 325.) <= 1 for d in gas))
      self.assertTrue(all(d['FrictionBrakeMode'] == 1 and d['FrictionBrakeCmd'] == 0 for d in brake))
      self.cc.gmResumeSetTest.active = False
      self.cc.actuators.accel = 2.  # model departure cannot defeat the hold latch
      for _ in range(8):
        output, _ = self.step()
      self.assertFalse(output.gmResumeSetActive)
      self.assertTrue(output.gmResumeSetHolding)
      self.assertEqual(output.gas, -540.)
      self.assertEqual(output.brake, 49.)
      last_gas = next(d for _, m, d in reversed(self.messages) if m[0] == 0x2CB)
      last_brake = next(d for _, m, d in reversed(self.messages) if m[0] == 0x315)
      self.assertEqual(last_gas['GasRegenFullStopActive'], 1)
      self.assertEqual(last_brake['FrictionBrakeMode'], 13)

  def test_factory_active_stops_pressing_but_roll_continues_until_release(self):
    self.press()
    for _ in range(8):
      self.step()
    self.cs.gmCruiseState = 1
    count = len(self.messages)
    for _ in range(60):
      output, _ = self.step()
    self.assertTrue(output.gmResumeSetActive)
    buttons = [d['ACCButtons'] for _, m, d in self.messages[count:] if m[0] == 0x1E1]
    self.assertTrue(buttons)
    self.assertTrue(all(b == 1 for b in buttons))

  def test_frozen_controlsd_heartbeat_reholds_at_card(self):
    self.press()
    for _ in range(30):
      output, _ = self.step(heartbeat=False)
    self.assertFalse(output.gmResumeSetActive)
    self.assertTrue(output.gmResumeSetHolding)
    self.assertGreater(output.brake, 0)
    for _ in range(30):
      output, _ = self.step()
    self.assertFalse(output.gmResumeSetActive)  # same press can never restart

  def test_speed_distance_and_time_bounds_at_card(self):
    for case, speed, steps, reason in (('speed', 2., 8, 'speedLimit'), ('distance', 1.5, 670, 'distanceLimit'),
                                       ('time', 0., 1010, 'timeout')):
      with self.subTest(case=case):
        self.setUp()
        self.press()
        self.cs.vEgo = self.cs.vEgoRaw = speed
        self.cs.standstill = speed == 0.
        for _ in range(steps):
          output, _ = self.step()
        self.assertFalse(output.gmResumeSetActive)
        self.assertTrue(output.gmResumeSetHolding)
        self.assertGreater(output.brake, 0)
        # The persistent latch may subsequently report requestRejected.
        self.assertIn(output.gmResumeSetReason, (reason, 'requestRejected'))

  def test_no_native_counter_does_not_transmit_blindly(self):
    self.press()
    for _ in range(20):
      self.step(native=False)
    count = len(self.messages)
    for _ in range(50):
      self.step(native=False)
    self.assertFalse(any(m[0] == 0x1E1 for _, m, _ in self.messages[count:]))

  def test_restarted_controller_rejects_inflight_active_request(self):
    self.press()
    for _ in range(50):
      self.step()
    self.gm = CarController(DBC[self.cp.carFingerprint], self.cp)
    for _ in range(8):
      output, _ = self.step()
    self.assertFalse(output.gmResumeSetActive)
    self.assertTrue(output.gmResumeSetHolding)
    self.assertGreater(output.brake, 0)

  def test_unsupported_controller_is_inert(self):
    self.cp.flags = 0
    self.gm.resume_set_test = ResumeSetActuation(self.cp)
    self.press()
    self.assertFalse(self.gm.resume_set_test.active)
    self.assertFalse(self.gm.resume_set_test.holding)

  def test_invalid_speed_aborts_without_breaking_brake_conversion(self):
    self.press()
    self.cs.vEgo = float('nan')
    for _ in range(8):
      output, _ = self.step()
    self.assertFalse(output.gmResumeSetActive)
    self.assertGreater(output.brake, 0)

  def test_button_echo_does_not_drive_sender_or_openpilot_buttons(self):
    from opendbc.car.gm.carstate import CarState, ButtonType
    self.press()
    for _ in range(4):
      self.step()
    self.vehicle.cruise_buttons = 2
    count = len(self.messages)
    for _ in range(10):
      self.step(native=False)
    self.assertFalse(any(m[0] == 0x1E1 for _, m, _ in self.messages[count:]))
    state = CarState(self.cp)
    parsers = state.get_can_parsers(self.cp)
    state.update(parsers)  # register lazily accessed messages
    state.resume_set_suppress_until = int((self.now + .5) * 1e9)
    for i, button in enumerate((1, 2, 1, 3, 1, 6)):
      ts = int((self.now + .03 * i) * 1e9)
      parsers[Bus.pt].update([ts, [gmcan.create_buttons(self.packer, 0, i % 4, button),
                                 self.packer.make_can_msg('AcceleratorPedal2', 0, {'CruiseState': 4})]])
      out = state.update(parsers)
      self.assertEqual(out.gmCruiseState, 4)
      self.assertEqual(state.cruise_buttons, button)
      self.assertFalse(state.update_button_enable(out.buttonEvents))
      self.assertFalse(any(b.type in (ButtonType.accelCruise, ButtonType.decelCruise) for b in out.buttonEvents))
      if button == 6:
        self.assertTrue(any(b.type == ButtonType.cancel and b.pressed for b in out.buttonEvents))
    # Physical Resume works normally again after the diagnostic echo window.
    for i, button in enumerate((1, 2)):
      parsers[Bus.pt].update([int((self.now + .6 + .03 * i) * 1e9), [gmcan.create_buttons(self.packer, 0, (6 + i) % 4, button)]])
      out = state.update(parsers)
    self.assertTrue(state.update_button_enable(out.buttonEvents))


if __name__ == '__main__':
  unittest.main()
