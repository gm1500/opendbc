#ifndef SAFETY_MODES_GM_H
#define SAFETY_MODES_GM_H

#include "opendbc/safety/declarations.h"

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
#define GM_COMMON_RX_CHECKS \
    {.msg = {{0x184, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0x34A, 0, 5, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0x1E1, 0, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0xBE, 0, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},    /* Volt, Silverado, Acadia Denali */ \
             {0xBE, 0, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},    /* Bolt EUV */ \
             {0xBE, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}}},  /* Escalade */ \
    {.msg = {{0x1C4, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{0xC9, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define GM_BTN_RESUME 2U
#define GM_BTN_SET 3U
#define GM_BTN_CANCEL 6U

// Both leases are checked at each TX, not just the 1 Hz safety tick.
static void gm_refresh_lateral_permission(void) {
  const uint32_t LEASE_US = 500000U;
  if (safety_get_ts_elapsed(safety_ts, safety.ts_lateral_rearm) >= LEASE_US) {
    safety.lateral_rearm_grace = false;
  }
  bool healthy = safety_setup.gm_lateral_only && safety.gm_gear_valid &&
                 !safety.relay_malfunction && !safety.steering_disengage &&
                 (safety_get_ts_elapsed(safety_ts, safety.ts_lateral_request) < LEASE_US);
  for (int i = 0; i < safety_setup.current_safety_config.rx_checks_len; i++) {
    const RxStatus *status = &safety.rx_status[i];
    healthy = healthy && status->msg_seen && status->valid_checksum && status->valid_quality_flag &&
              (status->wrong_counters < 5) &&
              (safety_get_ts_elapsed(safety_ts, status->last_timestamp) < LEASE_US);
  }
  // A permission that was granted cannot silently recover from a safety fault.
  if (safety.lateral_armed && safety.lateral_enabled && !healthy) {
    safety.lateral_suspended = true;
  }
  safety.lateral_allowed = healthy && safety.lateral_enabled && safety.lateral_requested && !safety.lateral_suspended &&
                           safety.gm_drive_gear && !safety.brake_pressed && !safety.regen_braking;
  if (safety.lateral_allowed) {
    safety.lateral_armed = true;
  }
}

static void gm_rx_hook(const CANPacket_t *msg) {
  const uint32_t GM_STANDSTILL_THRSLD = 10U;  // 0.311kph

  if (msg_matches(msg, 0x184U, 0U)) {
    int torque_driver_new = to_signed(GET_BYTES_BE(msg, 6, 2) & 0x7FFU, 11);
    // update array of samples
    update_sample(&safety.torque_driver, torque_driver_new);
  }

  // sample rear wheel speeds
  if (msg_matches(msg, 0x34AU, 0U)) {
    uint32_t left_rear_speed = GET_BYTES_BE(msg, 0, 2);
    uint32_t right_rear_speed = GET_BYTES_BE(msg, 2, 2);
    safety.vehicle_moving = (left_rear_speed > GM_STANDSTILL_THRSLD) || (right_rear_speed > GM_STANDSTILL_THRSLD);
  }

  // ACC steering wheel buttons (GM_CAM is tied to the PCM)
  if (msg_matches(msg, 0x1E1U, 0U) && !safety_setup.gm_pcm_cruise) {
    uint8_t button = (msg->data[5] & 0x70U) >> 4;

    // enter controls on falling edge of set or rising edge of resume (avoids fault)
    bool set = (button != GM_BTN_SET) && (button != GM_BTN_CANCEL) && (safety.cruise_button_prev == GM_BTN_SET);
    bool res = (button == GM_BTN_RESUME) && (safety.cruise_button_prev != GM_BTN_RESUME);
    if (set || res) {
      enter_controls();
      safety.lateral_suspended = false;
      if (safety_setup.gm_lateral_only) {
        // Allow the host to observe this physical button before relatching an
        // old host fault snapshot. Fault requests still deny lateral permission.
        safety.lateral_rearm_grace = true;
        safety.ts_lateral_rearm = safety_ts;
      }
    }

    // Cancel only exits cruise; independently requested lateral permission and
    // an existing lateral fault latch retain their state.
    if (button == GM_BTN_CANCEL) {
      exit_controls();
    }

    safety.cruise_button_prev = button;
  }

  if (safety_setup.gm_lateral_only && msg_matches(msg, 0x1F5U, 0U)) {
    const uint8_t gear = msg->data[3] & 0xFU;
    safety.gm_drive_gear = gear == 4U;
    safety.gm_gear_valid = gear <= 7U;
  }

  // Reference for brake pressed signals:
  // https://github.com/commaai/openpilot/blob/master/selfdrive/car/gm/carstate.py
  if (msg_matches(msg, 0xBEU, 0U) && (safety_setup.gm_hw == GM_ASCM)) {
    safety.brake_pressed = msg->data[1] >= 8U;
  }

  if (msg_matches(msg, 0xC9U, 0U) && (safety_setup.gm_hw == GM_CAM)) {
    safety.brake_pressed = GET_BIT(msg, 40U);
  }

  if (msg_matches(msg, 0x1C4U, 0U)) {
    safety.gas_pressed = msg->data[5] != 0U;

    // enter controls on rising edge of ACC, exit controls when ACC off
    if (safety_setup.gm_pcm_cruise) {
      bool cruise_engaged = (msg->data[1] >> 5) != 0U;
      pcm_cruise_check(cruise_engaged);
    }
  }

  if (msg_matches(msg, 0xBDU, 0U)) {
    safety.regen_braking = (msg->data[0] >> 4) != 0U;
  }
}

static bool gm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GM_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 10,
    .max_rate_down = 15,
    .driver_torque_allowance = 65,
    .driver_torque_multiplier = 4,
    .max_rt_delta = 128,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  gm_refresh_lateral_permission();

  // BRAKE: safety check
  if (msg->addr == 0x315U) {
    uint32_t brake = GET_BYTES_BE(msg, 0, 2) & 0xFFFU;
    brake = (0x1000U - brake) & 0xFFFU;
    if (longitudinal_brake_checks((int)brake, *safety_setup.gm_long_limits)) {
      tx = false;
    }
  }

  // LKA STEER: safety check
  if (msg->addr == 0x180U) {
    int desired_torque = to_signed(GET_BYTES_BE(msg, 0, 2) & 0x7FFU, 11);

    bool steer_req = GET_BIT(msg, 3U);

    if (steer_torque_cmd_checks_with_permission(desired_torque, steer_req, GM_STEERING_LIMITS,
                                                safety.controls_allowed || safety.lateral_allowed)) {
      tx = false;
    }
  }

  // GAS/REGEN: safety check
  if (msg->addr == 0x2CBU) {
    bool apply = GET_BIT(msg, 0U);
    // convert float CAN signal to an int for gas checks: 22534 / 0.125 = 180272
    int gas_regen = from_offset(GET_BYTES_BE(msg, 1, 3) & 0x7FFFFU, 180272);

    // Allow apply bit in pre-enabled and overriding states
    const bool apply_violation = !safety.controls_allowed && apply;
    const bool gas_violation = longitudinal_gas_checks(gas_regen, *safety_setup.gm_long_limits);
    bool violation = apply_violation || gas_violation;

    if (violation) {
      tx = false;
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation with stock longitudinal
  if (safety_setup.gm_pcm_cruise && (msg->addr == 0x1E1U)) {
    uint8_t button = (msg->data[5] >> 4) & 0x7U;

    bool allowed_cancel = (button == 6U) && safety.cruise_engaged_prev;
    if (!allowed_cancel) {
      tx = false;
    }
  }

  return tx;
}

static safety_config gm_init(uint16_t param) {
  const uint16_t GM_PARAM_HW_CAM = 1;
  const uint16_t GM_PARAM_EV = 4;
  const uint16_t GM_PARAM_HW_CAM_LONG = 2;
  const uint16_t GM_PARAM_SILVERADO_TORQUE = 8;
  const uint16_t GM_PARAM_LATERAL_ONLY = 16;

  // common safety checks assume unscaled integer values
  enum { GM_GAS_TO_CAN = 8 };  // 1 / 0.125

  static const LongitudinalLimits GM_ASCM_LONG_LIMITS = {
    .max_gas = 1018 * GM_GAS_TO_CAN,
    .min_gas = -650 * GM_GAS_TO_CAN,
    .inactive_gas = -650 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

  static const CanMsg GM_ASCM_TX_MSGS[] = {
    {0x180, 0, 4, .check_relay = true},
    {0x409, 0, 7, .check_relay = false},
    {0x40A, 0, 7, .check_relay = false},
    {0x2CB, 0, 8, .check_relay = true},
    {0x370, 0, 6, .check_relay = false},
    {0xA1, 1, 7, .check_relay = false},
    {0x306, 1, 8, .check_relay = false},
    {0x308, 1, 7, .check_relay = false},
    {0x310, 1, 2, .check_relay = false},
    {0x315, 2, 5, .check_relay = false},
  };

  static const LongitudinalLimits GM_CAM_LONG_LIMITS = {
    .max_gas = 1346 * GM_GAS_TO_CAN,
    .min_gas = -540 * GM_GAS_TO_CAN,
    .inactive_gas = -500 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

  // The higher calibrated torque command is limited to Silverado/Sierra in OP-long mode.
  static const LongitudinalLimits GM_SILVERADO_CAM_LONG_LIMITS = {
    .max_gas = 2450 * GM_GAS_TO_CAN,
    .min_gas = -540 * GM_GAS_TO_CAN,
    .inactive_gas = -500 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

#ifdef ALLOW_DEBUG
  // block PSCMStatus (0x184); forwarded through openpilot to hide an alert from the camera
  static const CanMsg GM_CAM_LONG_TX_MSGS[] = {
    {0x180, 0, 4, .check_relay = true},
    {0x315, 0, 5, .check_relay = true},
    {0x2CB, 0, 8, .check_relay = true},
    {0x370, 0, 6, .check_relay = true},
    {0x184, 2, 8, .check_relay = true},
  };
#endif

  static const RxCheck gm_rx_checks[] = {
    GM_COMMON_RX_CHECKS
  };

  static const RxCheck gm_lateral_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0x1F5, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const RxCheck gm_ev_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xBD, 0, 7, 40U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const CanMsg GM_CAM_TX_MSGS[] = {
    {0x180, 0, 4, .check_relay = true},
    {0x1E1, 2, 7, .check_relay = false},
    {0x184, 2, 8, .check_relay = true},
  };

  if (GET_FLAG(param, GM_PARAM_HW_CAM)) {
    safety_setup.gm_hw = GM_CAM;
    safety_setup.gm_long_limits = (GET_FLAG(param, GM_PARAM_SILVERADO_TORQUE) && GET_FLAG(param, GM_PARAM_HW_CAM_LONG))
                                   ? &GM_SILVERADO_CAM_LONG_LIMITS : &GM_CAM_LONG_LIMITS;
  } else {
    safety_setup.gm_hw = GM_ASCM;
    safety_setup.gm_long_limits = &GM_ASCM_LONG_LIMITS;
  }

  safety_setup.gm_pcm_cruise = (safety_setup.gm_hw == GM_CAM);

  safety_config ret;
  if (safety_setup.gm_hw == GM_CAM) {
    ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_TX_MSGS);
#ifdef ALLOW_DEBUG
    const bool gm_cam_long = GET_FLAG(param, GM_PARAM_HW_CAM_LONG);
    safety_setup.gm_pcm_cruise = !gm_cam_long;
    if (gm_cam_long) {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_LONG_TX_MSGS);
    }
#endif
  } else {
    ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_ASCM_TX_MSGS);
  }

  const bool gm_ev = GET_FLAG(param, GM_PARAM_EV);
  if (gm_ev) {
    SET_RX_CHECKS(gm_ev_rx_checks, ret);
  }

  safety_setup.gm_lateral_only = (safety_setup.gm_hw == GM_CAM) && !safety_setup.gm_pcm_cruise && !gm_ev &&
                                 GET_FLAG(param, GM_PARAM_SILVERADO_TORQUE | GM_PARAM_LATERAL_ONLY);
  if (safety_setup.gm_lateral_only) {
    SET_RX_CHECKS(gm_lateral_rx_checks, ret);
  }

  // ASCM does not forward any messages
  if (safety_setup.gm_hw == GM_ASCM) {
    ret.disable_forwarding = true;
  }
  return ret;
}

static const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
};

#endif  // SAFETY_MODES_GM_H
