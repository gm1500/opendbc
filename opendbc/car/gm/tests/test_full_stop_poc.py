import unittest
from types import SimpleNamespace as NS

from opendbc.can.dbc import DBC as Database
from opendbc.car import Bus, structs
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.values import CAR, DBC, CanBus


class TestSierraFullStopPOC(unittest.TestCase):
  def make_controller(self, fingerprint=CAR.CHEVROLET_SILVERADO, location='fwdCamera', direct_long=True):
    cp = structs.CarParams(carFingerprint=fingerprint, openpilotLongitudinalControl=direct_long,
                           networkLocation=location, radarUnavailable=True, mass=2586., wheelRadius=.419)
    controller = CarController(DBC[cp.carFingerprint], cp)
    db = Database(DBC[cp.carFingerprint][Bus.pt])
    state = NS(out=structs.CarState(), cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0,
               loopback_lka_steering_cmd_ts_nanos=0, pscm_status=dict.fromkeys(db.name_to_msg['PSCMStatus'].sigs, 0))
    return controller, state, db

  def commands(self, controller, state, db, *, speed=0., stopping=True, enabled=True):
    cc = structs.CarControl(enabled=enabled, longActive=enabled)
    cc.actuators.accel = -.37 if stopping else .3
    cc.actuators.longControlState = ('stopping' if stopping else 'pid') if enabled else 'off'
    state.out.vEgo = state.out.vEgoRaw = speed
    state.out.standstill = speed == 0.
    # Test each phase on a scheduled 25 Hz command frame.
    controller.frame = ((controller.frame + 3) // 4) * 4
    _, messages = controller.update(cc.as_reader(), state, int((100 + controller.frame * .01) * 1e9))
    commands = {}
    for addr, data, bus in messages:
      if addr not in (715, 789):
        continue
      values = {}
      for name, sig in db.addr_to_msg[addr].sigs.items():
        value = sig.get_raw_value(data)
        if sig.is_signed and value & (1 << (sig.size - 1)):
          value -= 1 << sig.size
        values[name] = value * sig.factor + sig.offset
      commands[addr] = (bus, values)
    return commands

  def test_approach_hold_release_and_rehold(self):
    controller, state, db = self.make_controller()
    for speed, stopping, brake_mode in ((1., True, 10), (.1, True, 10), (0., True, 13),
                                        (0., False, 1), (.2, False, 1), (0., True, 13)):
      with self.subTest(speed=speed, stopping=stopping):
        commands = self.commands(controller, state, db, speed=speed, stopping=stopping)
        gas_bus, gas = commands[715]
        brake_bus, brake = commands[789]
        self.assertEqual((gas_bus, brake_bus), (CanBus.POWERTRAIN, CanBus.POWERTRAIN))
        self.assertEqual(gas['GasRegenFullStopActive'], 0)
        self.assertEqual(gas['GasRegenCmdActive'], 1)
        self.assertEqual(brake['FrictionBrakeMode'], brake_mode)
        if stopping:
          self.assertEqual(gas['GasRegenCmd'], -540)
          self.assertLess(brake['FrictionBrakeCmd'], 0)
          if speed == 0.:
            self.assertEqual(brake['FrictionBrakeCmd'], -49)
        else:
          self.assertGreater(gas['GasRegenCmd'], 0)
          self.assertEqual(brake['FrictionBrakeCmd'], 0)

  def test_disengagement_retains_inactive_commands(self):
    controller, state, db = self.make_controller()
    self.commands(controller, state, db)
    commands = self.commands(controller, state, db, enabled=False)
    gas, brake = commands[715][1], commands[789][1]
    self.assertEqual(gas['GasRegenFullStopActive'], 0)
    self.assertEqual(gas['GasRegenCmdActive'], 0)
    self.assertEqual(gas['GasRegenCmd'], controller.params.INACTIVE_TORQUE)
    self.assertEqual(brake['FrictionBrakeCmd'], 0)
    self.assertEqual(brake['FrictionBrakeMode'], 1)

  def test_other_platforms_and_gateway_keep_full_stop(self):
    for fingerprint, location in ((CAR.CHEVROLET_SILVERADO, 'gateway'), (CAR.CHEVROLET_EQUINOX, 'fwdCamera')):
      with self.subTest(fingerprint=fingerprint, location=location):
        controller, state, db = self.make_controller(fingerprint, location)
        commands = self.commands(controller, state, db)
        self.assertEqual(commands[715][1]['GasRegenFullStopActive'], 1)
        self.assertEqual(commands[789][1]['FrictionBrakeMode'], 13)

  def test_stock_longitudinal_sends_no_gas_or_brake(self):
    controller, state, db = self.make_controller(direct_long=False)
    self.assertEqual(self.commands(controller, state, db), {})


if __name__ == '__main__':
  unittest.main()
