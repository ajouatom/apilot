#ifndef SAFETY_HYUNDAI_COMMON_H
#define SAFETY_HYUNDAI_COMMON_H

const uint8_t HYUNDAI_PREV_BUTTON_SAMPLES = 8;  // roughly 160 ms
const uint32_t HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph

enum {
  HYUNDAI_BTN_NONE = 0,
  HYUNDAI_BTN_RESUME = 1,
  HYUNDAI_BTN_SET = 2,
  HYUNDAI_BTN_CANCEL = 4,
};

// common state
bool hyundai_longitudinal = false;
uint8_t hyundai_last_button_interaction;  // button messages since the user pressed an enable button


void hyundai_common_cruise_state_check(const int cruise_engaged) {
  // some newer HKG models can re-enable after spamming cancel button,
  // so keep track of user button presses to deny engagement if no interaction

  // enter controls on rising edge of ACC and recent user button press, exit controls when ACC off
  if (!hyundai_longitudinal) {
    if (cruise_engaged && !cruise_engaged_prev && (hyundai_last_button_interaction < HYUNDAI_PREV_BUTTON_SAMPLES)) {
      controls_allowed = 1;
    }

    if (!cruise_engaged) {
      controls_allowed = 0;
    }
    cruise_engaged_prev = cruise_engaged;
  }
}

void hyundai_common_cruise_buttons_check(const int cruise_button, const int main_button) {
  if ((cruise_button == HYUNDAI_BTN_RESUME) || (cruise_button == HYUNDAI_BTN_SET) || (cruise_button == HYUNDAI_BTN_CANCEL) ||
      (main_button != 0)) {
    hyundai_last_button_interaction = 0U;
  } else {
    hyundai_last_button_interaction = MIN(hyundai_last_button_interaction + 1U, HYUNDAI_PREV_BUTTON_SAMPLES);
  }

  if (hyundai_longitudinal) {
    // exit controls on cancel press
    if (cruise_button == HYUNDAI_BTN_CANCEL) {
      controls_allowed = 0;
    }

    // enter controls on falling edge of resume or set
    bool set = (cruise_button == HYUNDAI_BTN_NONE) && (cruise_button_prev == HYUNDAI_BTN_SET);
    bool res = (cruise_button == HYUNDAI_BTN_NONE) && (cruise_button_prev == HYUNDAI_BTN_RESUME);
    if (set || res) {
      controls_allowed = 1;
    }

    cruise_button_prev = cruise_button;
  }
}

#endif
