import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.carstate import CarState, ButtonType
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, CruiseButtons, DBC


class TestGMLkaButton(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.CHEVROLET_SILVERADO)
    self.cs = CarState(self.CP)
    self.parsers = CarState.get_can_parsers(self.CP)
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.now = 1_000_000_000
    self.counter = 0
    self.cs.update(self.parsers)  # register lazily parsed messages

  def step(self, state=None, cruise=CruiseButtons.UNPRESS, bus=0):
    self.now += 10_000_000
    messages = []
    if state is not None:
      messages = [self.packer.make_can_msg('ASCMSteeringButton', bus, {
        'LKAButton': int(state), 'ACCButtons': cruise, 'RollingCounter': self.counter,
      })]
      self.counter = (self.counter + 1) % 4
    for cp in self.parsers.values():
      cp.update([(self.now, messages)])
    return self.cs.update(self.parsers)

  def events(self, state):
    return [(b.type, b.pressed) for b in state.buttonEvents if b.type == ButtonType.lkas]

  def test_unseen_and_other_buses_are_not_valid_off(self):
    self.assertFalse(self.step().gmLkaButtonValid)
    for bus in (2, 128):
      self.assertFalse(self.step(True, bus=bus).gmLkaButtonValid)

  def test_real_can_bit_and_first_observation_have_no_phantom_edge(self):
    data = self.packer.make_can_msg('ASCMSteeringButton', 0, {'LKAButton': 1})[1]
    self.assertEqual(data[2] & 0x80, 0x80)
    state = self.step(True)
    self.assertTrue(state.gmLkaButton)
    self.assertTrue(state.gmLkaButtonValid)
    self.assertEqual(self.events(state), [])

  def test_press_hold_release_and_cancel_are_independent(self):
    self.step(False)
    state = self.step(True, cruise=CruiseButtons.CANCEL)
    self.assertEqual(self.events(state), [(ButtonType.lkas, True)])
    self.assertIn((ButtonType.cancel, True), [(b.type, b.pressed) for b in state.buttonEvents])
    for _ in range(30):
      state = self.step(True)
      self.assertTrue(state.gmLkaButtonValid)
      self.assertEqual(self.events(state), [])
    self.assertEqual(self.events(self.step(False)), [(ButtonType.lkas, False)])

  def test_stale_retains_raw_value_but_loses_validity_and_does_not_fake_release(self):
    self.step(True)
    for _ in range(11):
      state = self.step()
      self.assertEqual(self.events(state), [])
    self.assertFalse(state.gmLkaButtonValid)
    self.assertTrue(state.gmLkaButton)
    state = self.step(False)
    self.assertTrue(state.gmLkaButtonValid)
    self.assertFalse(state.gmLkaButton)
    self.assertEqual(self.events(state), [])

  def test_delayed_iteration_does_not_make_cached_bit_fresh(self):
    self.step(True)
    self.now += 1_000_000_000
    wheel = self.packer.make_can_msg('EBCMWheelSpdRear', 0, {})
    self.parsers[Bus.pt].update([(self.now, [wheel])])
    state = self.cs.update(self.parsers)
    self.assertFalse(state.gmLkaButtonValid)
    self.assertTrue(state.gmLkaButton)
