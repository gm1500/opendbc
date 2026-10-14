"""Bounded, driver-held Sierra diagnostic. No automatic resume or speed-set control."""
import math

from opendbc.car.gm import gmcan
from opendbc.car.gm.values import CAR, CanBus, CruiseButtons, GMFlags, GMSafetyFlags

MAX_DURATION = 10.0
MAX_SPEED = 2.0  # m/s, 7.2 km/h; an abort threshold, not a speed target
MAX_DISTANCE = 10.0  # m; movement during the attempt, excludes subsequent stopping distance
TEST_ACCEL = 0.3
REHOLD_ACCEL = -0.8
HEARTBEAT_MAX_AGE = 0.20
BUTTON_MAX_AGE = 0.10
BUTTON_PHASE_DELAY = 0.01
BUTTON_PHASE_END = 0.025
BUTTON_MIN_INTERVAL = 0.025
BUTTON_REPEAT_INTERVAL = 0.50
BUTTON_PRESS_FRAMES = 5
BUTTON_RELEASE_FRAMES = 2


def supported(CP):
  flags = GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.RESUME_SET_TEST
  return (CP.carFingerprint == CAR.CHEVROLET_SILVERADO and CP.openpilotLongitudinalControl and
          str(CP.networkLocation) == 'fwdCamera' and bool(CP.flags & GMFlags.RESUME_SET_TEST) and
          len(CP.safetyConfigs) == 1 and str(CP.safetyConfigs[0].safetyModel) == 'gm' and
          CP.safetyConfigs[0].safetyParam == flags)


class ResumeSetActuation:
  """Second watchdog at card/CarController, independent of controlsd's state machine."""
  def __init__(self, CP):
    self.supported = supported(CP)
    self.active = self.holding = self.recovered = False
    self.request_id = self.last_seen_id = 0
    self.button = 0
    self.start = self.last_update = 0.
    self.distance = 0.
    self.reason = 'idle'
    self.tx_count = 0
    self.last_tx = -1.
    self.last_observation = 0
    self.burst_start = -1.
    self.burst_index = 0
    self.release_pending = 0
    self.release_until = 0.

  def update(self, CC, CS, now, brake_was_held):
    req, cs = CC.gmResumeSetTest, CS.out
    if not self.supported:
      return
    new_id = req.requestId > self.last_seen_id
    if new_id:
      self.last_seen_id = req.requestId  # a rejected request is never queued
    if not CC.enabled or not CC.longActive or cs.gasPressed or cs.brakePressed:
      if self.active:
        self.release_pending = BUTTON_RELEASE_FRAMES
        self.release_until = now + HEARTBEAT_MAX_AGE
      self.active = self.holding = False
      self.reason = 'driver'
      return

    finite = all(math.isfinite(v) for v in (cs.vEgo, cs.vEgoRaw, cs.aEgo, CC.actuators.accel))
    valid = (finite and cs.canValid and not cs.canTimeout and not cs.accFaulted and not cs.carNotReady and
             not cs.carFaultedNonCritical and not cs.doorOpen and not cs.seatbeltUnlatched and not cs.parkingBrake and
             not cs.stockAeb and not cs.stockFcw and str(cs.gearShifter) == 'drive' and cs.gmCruiseState in (1, 2, 4))
    valid = valid and 0 <= now - CS.buttons_ts_nanos * 1e-9 <= BUTTON_MAX_AGE
    fresh = 0 <= now - req.heartbeatMonoTime * 1e-9 <= HEARTBEAT_MAX_AGE
    held = req.active and fresh and req.button in (CruiseButtons.RES_ACCEL, CruiseButtons.DECEL_SET)
    if self.active:
      dt = now - self.last_update
      if finite and 0 <= dt <= HEARTBEAT_MAX_AGE:
        self.distance += max(abs(cs.vEgo), abs(cs.vEgoRaw)) * dt
      self.recovered |= cs.gmCruiseState == 1
      reason = ('vehicleBlocked' if not valid else 'released' if not held else
                'requestChanged' if req.requestId != self.request_id or req.button != self.button else
                'controlStale' if not 0 <= dt <= HEARTBEAT_MAX_AGE else
                'speedLimit' if max(abs(cs.vEgo), abs(cs.vEgoRaw)) >= MAX_SPEED else
                'reverse' if min(cs.vEgo, cs.vEgoRaw) < -0.05 else
                'distanceLimit' if self.distance >= MAX_DISTANCE else
                'timeout' if not 0 <= now - self.start < MAX_DURATION else None)
      if reason:
        self.active = False
        self.release_pending = BUTTON_RELEASE_FRAMES
        self.release_until = now + HEARTBEAT_MAX_AGE
        self.reason = reason
    elif (new_id and held and valid and brake_was_held and cs.standstill and cs.gmCruiseState == 4 and
          max(abs(cs.vEgo), abs(cs.vEgoRaw)) < 0.05 and 0 <= now - req.requestId * 1e-9 <= HEARTBEAT_MAX_AGE):
      self.active = self.holding = True
      self.recovered = False
      self.request_id, self.button = req.requestId, req.button
      self.start, self.distance = now, 0.
      self.reason = 'rolling'
      self.burst_start, self.burst_index = -1., 0
      self.release_pending = 0
      self.tx_count = 0
    elif req.holding or req.active:
      # A stale/rejected active request must never fall through to positive
      # normal actuation, including a controller restart during an attempt.
      self.holding = True
      self.reason = 'requestRejected' if req.active else 'reholding'

    if self.holding:
      CS.resume_set_suppress_until = int((now + 0.5) * 1e9)
    self.last_update = now

  def buttons(self, packer, CS, now):
    if not self.supported:
      return []
    if self.active and self.recovered:
      if self.burst_index:
        self.release_pending = max(self.release_pending, BUTTON_RELEASE_FRAMES)
        self.release_until = now + HEARTBEAT_MAX_AGE
        self.burst_index = 0
      self.reason = 'cruiseActive'
    sending = self.active and not self.recovered
    if now > self.release_until:
      self.release_pending = 0
    if not sending and not self.release_pending:
      return []
    ts = CS.buttons_ts_nanos
    age = now - ts * 1e-9
    # Use fresh neutral native frames only. An echoed injected press must not
    # drive the next counter or generate a self-sustaining feedback sequence.
    if (CS.cruise_buttons != CruiseButtons.UNPRESS or ts == self.last_observation or
        not BUTTON_PHASE_DELAY <= age <= BUTTON_PHASE_END or now - self.last_tx < BUTTON_MIN_INTERVAL):
      return []

    if self.release_pending:
      button = CruiseButtons.UNPRESS
      self.release_pending -= 1
    else:
      if self.burst_start < 0 or (self.burst_index >= BUTTON_PRESS_FRAMES + BUTTON_RELEASE_FRAMES and
                                  now - self.burst_start >= BUTTON_REPEAT_INTERVAL):
        self.burst_start, self.burst_index = now, 0
      if self.burst_index >= BUTTON_PRESS_FRAMES + BUTTON_RELEASE_FRAMES:
        return []
      button = self.button if self.burst_index < BUTTON_PRESS_FRAMES else CruiseButtons.UNPRESS
      self.burst_index += 1
    self.last_tx, self.last_observation = now, ts
    self.tx_count += 1
    CS.resume_set_suppress_until = int((now + 0.5) * 1e9)
    return [gmcan.create_buttons(packer, CanBus.POWERTRAIN, (CS.buttons_counter + 1) % 4, button)]
