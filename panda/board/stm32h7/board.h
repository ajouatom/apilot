// ///////////////////////////////////////////////////////////// //
// Hardware abstraction layer for all different supported boards //
// ///////////////////////////////////////////////////////////// //
#include "boards/board_declarations.h"
#include "boards/unused_funcs.h"

// ///// Board definition and detection ///// //
#include "drivers/harness.h"
#include "drivers/fan.h"
#include "stm32h7/llfan.h"
#include "stm32h7/llrtc.h"
#include "drivers/rtc.h"
#include "boards/red.h"
#include "boards/red_chiplet.h"
#include "boards/red_v2.h"
#include "boards/tres.h"


uint8_t get_board_id(void) {
  return detect_with_pull(GPIOF, 7, PULL_UP) |
         (detect_with_pull(GPIOF, 8, PULL_UP) << 1U) |
         (detect_with_pull(GPIOF, 9, PULL_UP) << 2U) |
         (detect_with_pull(GPIOF, 10, PULL_UP) << 3U);
}

void detect_board_type(void) {
  const uint8_t board_id = get_board_id();

  if (board_id == 0U) {
    hw_type = HW_TYPE_RED_PANDA;
    current_board = &board_red;
  } else if (board_id == 1U) {
    hw_type = HW_TYPE_RED_PANDA_V2;
    current_board = &board_red_v2;
  } else if (board_id == 2U) {
    hw_type = HW_TYPE_TRES;
    current_board = &board_tres;
  } else {
    hw_type = HW_TYPE_UNKNOWN;
    puts("Hardware type is UNKNOWN!\n");
  }
}

bool has_external_debug_serial = 0;
void detect_external_debug_serial(void) {
  // detect if external serial debugging is present
  has_external_debug_serial = detect_with_pull(GPIOA, 3, PULL_DOWN) || detect_with_pull(GPIOE, 7, PULL_DOWN);
}
