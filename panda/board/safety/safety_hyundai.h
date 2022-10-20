#include "safety_hyundai_common.h"

const SteeringLimits HYUNDAI_STEERING_LIMITS = {
  .max_steer = 409,
  .max_rt_delta = 112,
  .max_rt_interval = 250000,
  .max_rate_up = 3,
  .max_rate_down = 7,
  .driver_torque_allowance = 50,
  .driver_torque_factor = 2,
  .type = TorqueDriverLimited,

  // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
  // we allow setting CF_Lkas_ActToi bit to 0 while maintaining the requested torque value for two consecutive frames
  .min_valid_request_frames = 89,
  .max_invalid_request_frames = 2,
  .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
  .has_steer_req_tolerance = true,
};

#ifdef CANFD
#else
bool Lcan_bus1 = false;
bool Fwd_bus1 = false;
bool Fwd_obd = false;
bool Fwd_bus2 = true;
int OBD_cnt = 20;
int LKAS11_bus0_cnt = 0;
int Lcan_bus1_cnt = 0;
int MDPS12_checksum = -1;
int MDPS12_cnt = 0;
int Last_StrColTq = 0;

int LKAS11_op = 0;
int MDPS12_op = 0;
int CLU11_op = 0;
int SCC12_op = 0;
int SCC12_car = 0;
int EMS11_op = 0;
int MDPS_bus = -1;
int SCC_bus = -1;
#endif


const int HYUNDAI_MAX_ACCEL = 200;  // 1/100 m/s2
const int HYUNDAI_MIN_ACCEL = -350; // 1/100 m/s2

const CanMsg HYUNDAI_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 0, 4}, // CLU11 Bus 0
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
};

const CanMsg HYUNDAI_LONG_TX_MSGS[] = {
  {593, 2, 8},                              // MDPS12, Bus 2
  {790, 1, 8},                              // EMS11, Bus 1
  {832, 0, 8}, {832, 1, 8}, // LKAS11 Bus 0,1
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11 Bus 0,1,2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, // SCC11 Bus 0
  {1057, 0, 8}, // SCC12 Bus 0
  {1290, 0, 8}, // SCC13 Bus 0
  {905, 0, 8},  // SCC14 Bus 0
  {1186, 0, 2}, // FRT_RADAR11 Bus 0
  {909, 0, 8},  // FCA11 Bus 0
  {1155, 0, 8}, // FCA12 Bus 0
  {2000, 0, 8}, // radar UDS TX addr Bus 0 (for radar disable)
};
const CanMsg HYUNDAI_CAMERA_SCC_LONG_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 2, 4}, // CLU11 Bus 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, // SCC11 Bus 0
  {1057, 0, 8}, // SCC12 Bus 0
  {1290, 0, 8}, // SCC13 Bus 0
  {905, 0, 8},  // SCC14 Bus 0
  {909, 0, 8},  // FCA11 Bus 0
  {1155, 0, 8}, // FCA12 Bus 0
};

const CanMsg HYUNDAI_CAMERA_SCC_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 2, 4}, // CLU11 Bus 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
};

AddrCheckStruct hyundai_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_ADDR_CHECK_LEN (sizeof(hyundai_addr_checks) / sizeof(hyundai_addr_checks[0]))

AddrCheckStruct hyundai_cam_scc_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_CAM_SCC_ADDR_CHECK_LEN (sizeof(hyundai_cam_scc_addr_checks) / sizeof(hyundai_cam_scc_addr_checks[0]))

AddrCheckStruct hyundai_long_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1265, 0, 4, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LONG_ADDR_CHECK_LEN (sizeof(hyundai_long_addr_checks) / sizeof(hyundai_long_addr_checks[0]))

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LEGACY_ADDR_CHECK_LEN (sizeof(hyundai_legacy_addr_checks) / sizeof(hyundai_legacy_addr_checks[0]))

AddrCheckStruct hyundai_legacy_long_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1265, 0, 4, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LEGACY_LONG_ADDR_CHECK_LEN (sizeof(hyundai_legacy_long_addr_checks) / sizeof(hyundai_legacy_long_addr_checks[0]))
const int HYUNDAI_PARAM_CAMERA_SCC = 8;

bool hyundai_legacy = false;
bool hyundai_camera_scc = false;
bool hyundai_scc_bus2 = false;

addr_checks hyundai_rx_checks = {hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};

static uint8_t hyundai_get_counter(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 608) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3U;
  } else if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7U;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 1265) {
    cnt = (GET_BYTE(to_push, 3) >> 4) & 0xFU;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint32_t hyundai_get_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 608) {
    chksum = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 902) {
    chksum = ((GET_BYTE(to_push, 7) >> 6) << 2) | (GET_BYTE(to_push, 5) >> 6);
  } else if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xFU;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint32_t hyundai_compute_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  if (addr == 902) {
    // count the bits
    for (int i = 0; i < 8; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      for (int j = 0; j < 8; j++) {
        uint8_t bit = 0;
        // exclude checksum and counter
        if (((i != 1) || (j < 6)) && ((i != 3) || (j < 6)) && ((i != 5) || (j < 6)) && ((i != 7) || (j < 6))) {
          bit = (b >> (uint8_t)j) & 1U;
        }
        chksum += bit;
      }
    }
    chksum = (chksum ^ 9U) & 15U;
  } else {
    // sum of nibbles
    for (int i = 0; i < 8; i++) {
      if ((addr == 916) && (i == 7)) {
        continue; // exclude
      }
      uint8_t b = GET_BYTE(to_push, i);
      if (((addr == 608) && (i == 7)) || ((addr == 916) && (i == 6)) || ((addr == 1057) && (i == 7))) {
        b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
      }
      chksum += (b % 16U) + (b / 16U);
    }
    chksum = (16U - (chksum %  16U)) % 16U;
  }

  return chksum;
}

static int hyundai_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &hyundai_rx_checks,
                                 hyundai_get_checksum, hyundai_compute_checksum,
                                 hyundai_get_counter);

  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

#ifndef CANFD
//#if 0
  if (!valid) { puts("  CAN RX invalid: "); puth(addr); puts("\n"); }
  if (bus == 1 && Lcan_bus1) { valid = false; }

  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) {
      Lcan_bus1_cnt = 500;
      if (Fwd_bus1 || !Lcan_bus1) { Lcan_bus1 = true; Fwd_bus1 = false; puts("  LCAN on bus1: forwarding disabled\n"); }
  }
  // check if LKAS11 on Bus0
  if (addr == 832) {
      if (bus == 0 && Fwd_bus2) { Fwd_bus2 = false; LKAS11_bus0_cnt = 20; puts("  LKAS11 on bus0: forwarding disabled\n"); }
      if (bus == 2) {
          if (LKAS11_bus0_cnt > 0) { LKAS11_bus0_cnt--; }
          else if (!Fwd_bus2) { Fwd_bus2 = true; puts("  LKAS11 on bus2: forwarding enabled\n"); }
          if (Lcan_bus1_cnt > 0) { Lcan_bus1_cnt--; }
          else if (Lcan_bus1) { Lcan_bus1 = false; puts("  Lcan not on bus1\n");}
      }
  }
  // check MDPS12 or MDPS11 on Bus
  if ((addr == 593 || addr == 897) && MDPS_bus != bus) {
      if (bus != 1 || (!Lcan_bus1 || Fwd_obd)) {
          MDPS_bus = bus;
          if (bus == 1 && !Fwd_obd) {
              puts("  MDPS on bus1\n");
              if (!Fwd_bus1 && !Lcan_bus1) { Fwd_bus1 = true; puts("  bus1 forwarding enabled\n"); }
          }
          else if (bus == 1) { puts("  MDPS on obd bus\n"); }
      }
  }
  // check SCC11 or SCC12 on Bus
  if ((addr == 1056 || addr == 1057) && SCC_bus != bus) {
      if (bus != 1 || !Lcan_bus1) {
          SCC_bus = bus;
          if (bus == 1) {
              hyundai_scc_bus2 = true;
              puts("  SCC on bus1\n");
              if (!Fwd_bus1) { Fwd_bus1 = true; puts("  bus1 forwarding enabled\n"); }
          }
          if (bus == 2) { 
              hyundai_scc_bus2 = true;
              puts("  SCC bus = bus2\n");
          }
      }
  }
#endif

  // SCC12 is on bus 2 for camera-based SCC cars, bus 0 on all others
  if (valid && (addr == 1057) && (((bus == 0) && !hyundai_camera_scc) || ((bus == 2) && hyundai_camera_scc))) {
    // 2 bits: 13-14
    int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3U;
    hyundai_common_cruise_state_check(cruise_engaged);
  }
#ifndef CANFD
  // MDPS12
  if (valid) {
      if (addr == 593 && bus == MDPS_bus) {
          int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
          // update array of samples
          update_sample(&torque_driver, torque_driver_new);
      }
  }
#endif
  if (valid && (bus == 0)) {
#ifdef CANFD
    if (addr == 593) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }
#endif

    // ACC steering wheel buttons
    if (addr == 1265) {
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      int main_button = GET_BIT(to_push, 3U);
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((addr == 881) && hyundai_ev_gas_signal) {
      gas_pressed = (((GET_BYTE(to_push, 4) & 0x7FU) << 1) | GET_BYTE(to_push, 3) >> 7) != 0U;
    } else if ((addr == 881) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BYTE(to_push, 7) != 0U;
    } else if ((addr == 608) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
    } else {
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 902) {
      uint32_t hyundai_speed = (GET_BYTES_04(to_push) & 0x3FFFU) + ((GET_BYTES_48(to_push) >> 16) & 0x3FFFU);  // FL + RR
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    if (addr == 916) {
      //ajouatom: brake_pressed = GET_BIT(to_push, 55U) != 0U;
    }

    bool stock_ecu_detected = (addr == 832);

    // If openpilot is controlling longitudinal we need to ensure the radar is turned off
    // Enforce by checking we don't see SCC12
    if (hyundai_longitudinal && (addr == 1057)) {
      stock_ecu_detected = true;
    }
    generic_rx_checks(stock_ecu_detected);
  }
  return valid;
}

static int hyundai_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (hyundai_longitudinal && hyundai_camera_scc) {
      tx = msg_allowed(to_send, HYUNDAI_CAMERA_SCC_LONG_TX_MSGS, sizeof(HYUNDAI_CAMERA_SCC_LONG_TX_MSGS) / sizeof(HYUNDAI_CAMERA_SCC_LONG_TX_MSGS[0]));
  }
  else if (hyundai_longitudinal) {
      tx = msg_allowed(to_send, HYUNDAI_LONG_TX_MSGS, sizeof(HYUNDAI_LONG_TX_MSGS)/sizeof(HYUNDAI_LONG_TX_MSGS[0]));
  } else if (!hyundai_longitudinal && hyundai_camera_scc) {
    tx = msg_allowed(to_send, HYUNDAI_CAMERA_SCC_TX_MSGS, sizeof(HYUNDAI_CAMERA_SCC_TX_MSGS)/sizeof(HYUNDAI_CAMERA_SCC_TX_MSGS[0]));
  } else {
    tx = msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]));
  }

  // FCA11: Block any potential actuation
  //if (addr == 909 && !hyundai_scc_bus2) {
  if (addr == 909) {
    int CR_VSM_DecCmd = GET_BYTE(to_send, 1);
    int FCA_CmdAct = GET_BIT(to_send, 20U);
    int CF_VSM_DecCmdAct = GET_BIT(to_send, 31U);

    if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
      tx = 0;
    }
  }

  // ACCEL: safety check
  if (addr == 1057) {
    int desired_accel_raw = (((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) - 1023U;
    int desired_accel_val = ((GET_BYTE(to_send, 5) << 3) | (GET_BYTE(to_send, 4) >> 5)) - 1023U;

    int aeb_decel_cmd = GET_BYTE(to_send, 2);
    int aeb_req = GET_BIT(to_send, 54U);

    bool violation = 0;

    if (!longitudinal_allowed) {
      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = 1;
      }
    }
    violation |= max_limit_check(desired_accel_raw, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);
    violation |= max_limit_check(desired_accel_val, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);

    violation |= (aeb_decel_cmd != 0);
    violation |= (aeb_req != 0);

    if (violation) {
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = 1;//ajouatom GET_BIT(to_send, 27U) != 0U;

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_STEERING_LIMITS)) {
      tx = 0;
    }
  }

  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if ((addr == 2000) && !hyundai_camera_scc && !hyundai_scc_bus2) {  // ajouatom
    if ((GET_BYTES_04(to_send) != 0x00803E02U) || (GET_BYTES_48(to_send) != 0x0U)) {
      tx = 0;
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation
  if ((addr == 1265) && !hyundai_longitudinal) {
    int button = GET_BYTE(to_send, 0) & 0x7U;

    bool allowed_resume = (button == 1) && controls_allowed;
    bool allowed_cancel = (button == 4) && cruise_engaged_prev;
    if (!(allowed_resume || allowed_cancel)) {
      tx = 0;
    }
  }
#ifndef CANFD
  //int bus = GET_BUS(to_send);
  //if (addr == 593) { MDPS12_op = 20; }
  //if (addr == 1265 && bus == 1) { CLU11_op = 20; } // only count mesage created for MDPS
  //if (addr == 1057) { SCC12_op = 20; if (SCC12_car > 0) { SCC12_car -= 1; } }
  //if (addr == 790) { EMS11_op = 20; }
#endif

  return tx;
}
int _test_count = 100;
static int hyundai_fwd_hook(int bus_num, CANPacket_t *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  int is_lkas11_msg = (addr == 832);
  int is_lfahda_mfc_msg = (addr == 1157);
  int is_scc_msg = (addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905);
  int is_fca_msg = (addr == 909) || (addr == 1155);

  //int is_clu11_msg = (addr == 1265);
  //int is_mdps12_msg = (addr = 593);
  //int is_ems11_msg = (addr == 790);
// 로직에 뭔가 문제가 있는듯~~~
//  일단... 정상적인 포워딩으로 가자`
//#ifndef CANFD
#if 0
  int fwd_to_bus1 = -1;
  if (Fwd_bus1 || Fwd_obd) { fwd_to_bus1 = 1; }
  // forward cam to ccan and viceversa, except lkas cmd
  if (_test_count > 0) _test_count--;
  if (_test_count == 0) {
      _test_count = 200;
      //puts("test="); 
      //puth(Fwd_bus2); puts(",");   // 1
      //puth(CLU11_op); puts(",");   // 0
      //puth(MDPS12_op); puts(",");   // 0
      //puth(Fwd_bus1); puts(",");   // 0
      //puth(Fwd_obd); puts(",");    // 0
      //puth(SCC12_op); puts(",");    // SCCxx
      //puth(fwd_to_bus1); puts(","); // 0     
      //puth(LKAS11_op); puts("\n");  // 0
  }
  if (Fwd_bus2) {
      if (bus_num == 0) {
          if (CLU11_op>0 || addr != 1265 || MDPS_bus == 0) {
              if (MDPS12_op>0 || addr != 593) {
                  if (EMS11_op>0 || addr != 790) { bus_fwd = fwd_to_bus1 == 1 ? 12 : 2; }
                  else { 
                      bus_fwd = 2;  // OP create EMS11 for MDPS
                      if(EMS11_op > 0) EMS11_op --;
                  }
              }
              else {
                  bus_fwd = fwd_to_bus1;  // OP create MDPS for LKAS
                  if(MDPS12_op > 0) MDPS12_op --;
              }
          }
          else {
              bus_fwd = 2; // OP create CLU12 for MDPS
              if(CLU11_op > 0) CLU11_op --;
          }
          //if (bus_fwd != 2) {
              // 0x251만 bus: 2
              //puts("bus not 2"); puth(bus_fwd); puts(","); puth(addr);  puts("\n");
          //}
          //bus_fwd = 2;
      }
      if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
          if (MDPS12_op>0 || addr != 593) {
              if (SCC12_op>0 || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
                  bus_fwd = 20;
              }
              else {
                  bus_fwd = 2;  // OP create SCC11 SCC12 SCC13 SCC14 for Car
                  if(SCC12_op > 0) SCC12_op --;
              }
          }
          else {
              bus_fwd = 0;  // OP create MDPS for LKAS
              if(MDPS12_op > 0) MDPS12_op --;
          }
          puts("bus_num:1 "); puth(addr); puts(","); puth(bus_fwd); puts("\n");
      }
      //if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
      //    if ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905));
      //    else bus_fwd = 0;
      //}
      if (bus_num == 2) {
          if (LKAS11_op > 0 || (addr != 832 && addr != 1157)) {
              
              if (SCC12_op > 0 || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
                  bus_fwd = fwd_to_bus1 == 1 ? 10 : 0;
              }
              else {
                  bus_fwd = fwd_to_bus1;  // OP create SCC12 for Car
                  if(SCC12_op>0) SCC12_op --;
              }
          }
          else if (MDPS_bus == 0) {
              bus_fwd = fwd_to_bus1; // OP create LKAS and LFA for Car
              if(LKAS11_op > 0) LKAS11_op --;
          }
          else {
              if (LKAS11_op > 0) LKAS11_op --; // OP create LKAS and LFA for Car and MDPS
          }
      }
  }
  else {
      if (bus_num == 0) {
          bus_fwd = fwd_to_bus1;
      }
      if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
          bus_fwd = 0;
      }
  }

// Debug Codes
  if (bus_num == 0 && bus_fwd != 2) {
      puts("Bus0: fwd="); puth(bus_fwd); puts(",addr="); puth(addr); puts("\n");
      bus_fwd = 2;
      
  }
  if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
      if ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905)) {
          if (bus_fwd != -1) {
              puts("Bus-1: fwd="); puth(bus_fwd); puts(",addr="); puth(addr); puts(",SCC12_op="); puth(SCC12_op); puts("\n");
          }
      }
      else {
          if (bus_fwd != 0) {
              puts("Bus2: fwd="); puth(bus_fwd); puts(",addr="); puth(addr); puts("\n");
          }
      }
  }
#else
  // forward cam to ccan and viceversa, except lkas cmd
  if (bus_num == 0) {
    bus_fwd = 2;
  }
  //if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
  //    if ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905));
  //    else bus_fwd = 0;
  //}
  if (bus_num == 2) {

      int block_msg = 0;
      if (hyundai_longitudinal && hyundai_camera_scc) block_msg = is_lkas11_msg || is_lfahda_mfc_msg || is_scc_msg || is_fca_msg;
      else block_msg = is_lkas11_msg || is_lfahda_mfc_msg || is_scc_msg;

      if (!block_msg) {
          bus_fwd = 0;
      }
  }

#endif

  return bus_fwd;
}

static const addr_checks* hyundai_init(uint16_t param) {
  hyundai_common_init(param);
  hyundai_legacy = false;
  hyundai_camera_scc = GET_FLAG(param, HYUNDAI_PARAM_CAMERA_SCC);
  hyundai_scc_bus2 = false;

  //if (hyundai_camera_scc) {
  //  hyundai_longitudinal = false;
  //}

  if (hyundai_longitudinal) {
    hyundai_rx_checks = (addr_checks){hyundai_long_addr_checks, HYUNDAI_LONG_ADDR_CHECK_LEN};
  } else if (hyundai_camera_scc) {
    hyundai_rx_checks = (addr_checks){hyundai_cam_scc_addr_checks, HYUNDAI_CAM_SCC_ADDR_CHECK_LEN};
  } else {
    hyundai_rx_checks = (addr_checks){hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};
  }
  return &hyundai_rx_checks;
}

static const addr_checks* hyundai_legacy_init(uint16_t param) {
  hyundai_common_init(param);
  hyundai_legacy = true;
  hyundai_camera_scc = false;
  hyundai_scc_bus2 = false;
  if (hyundai_longitudinal) {
      hyundai_rx_checks = (addr_checks){ hyundai_legacy_long_addr_checks, HYUNDAI_LEGACY_LONG_ADDR_CHECK_LEN };
  }
  else {
      hyundai_rx_checks = (addr_checks){ hyundai_legacy_addr_checks, HYUNDAI_LEGACY_ADDR_CHECK_LEN };
  }
  return &hyundai_rx_checks;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};
