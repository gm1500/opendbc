import unittest

from opendbc.car.gm.carstate import update_lka_button_latch


class TestLkaButtonLatch(unittest.TestCase):
  def test_latches_only_on_rising_edges(self):
    pressed = False
    latched = False

    pressed, latched = update_lka_button_latch(pressed, latched, False)
    self.assertEqual((pressed, latched), (False, False))

    pressed, latched = update_lka_button_latch(pressed, latched, True)
    self.assertEqual((pressed, latched), (True, True))

    pressed, latched = update_lka_button_latch(pressed, latched, True)
    self.assertEqual((pressed, latched), (True, True))

    pressed, latched = update_lka_button_latch(pressed, latched, False)
    self.assertEqual((pressed, latched), (False, True))

    pressed, latched = update_lka_button_latch(pressed, latched, True)
    self.assertEqual((pressed, latched), (True, False))
