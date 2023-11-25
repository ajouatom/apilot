#pragma once

#include <map>
#include <memory>
#include <string>

#include <QObject>
#include <QTimer>
#include <QColor>
#include <QFuture>
#include <QPolygonF>
#include <QTransform>
#include "nanovg.h"

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/timing.h"
#include "system/hardware/hw.h"

const int UI_BORDER_SIZE = 30;
const int UI_HEADER_HEIGHT = 420;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;
typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int centerY() const { return y + h / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

const float MIN_DRAW_DISTANCE = 10.0;
const float MAX_DRAW_DISTANCE = 100.0;
constexpr mat3 DEFAULT_CALIBRATION = {{ 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0 }};
constexpr mat3 FCAM_INTRINSIC_MATRIX = (mat3){{2648.0, 0.0, 1928.0 / 2,
                                           0.0, 2648.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};
// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
constexpr mat3 ECAM_INTRINSIC_MATRIX = (mat3){{567.0, 0.0, 1928.0 / 2,
                                           0.0, 567.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};


constexpr vec3 default_face_kpts_3d[] = {
  {-5.98, -51.20, 8.00}, {-17.64, -49.14, 8.00}, {-23.81, -46.40, 8.00}, {-29.98, -40.91, 8.00}, {-32.04, -37.49, 8.00},
  {-34.10, -32.00, 8.00}, {-36.16, -21.03, 8.00}, {-36.16, 6.40, 8.00}, {-35.47, 10.51, 8.00}, {-32.73, 19.43, 8.00},
  {-29.30, 26.29, 8.00}, {-24.50, 33.83, 8.00}, {-19.01, 41.37, 8.00}, {-14.21, 46.17, 8.00}, {-12.16, 47.54, 8.00},
  {-4.61, 49.60, 8.00}, {4.99, 49.60, 8.00}, {12.53, 47.54, 8.00}, {14.59, 46.17, 8.00}, {19.39, 41.37, 8.00},
  {24.87, 33.83, 8.00}, {29.67, 26.29, 8.00}, {33.10, 19.43, 8.00}, {35.84, 10.51, 8.00}, {36.53, 6.40, 8.00},
  {36.53, -21.03, 8.00}, {34.47, -32.00, 8.00}, {32.42, -37.49, 8.00}, {30.36, -40.91, 8.00}, {24.19, -46.40, 8.00},
  {18.02, -49.14, 8.00}, {6.36, -51.20, 8.00}, {-5.98, -51.20, 8.00},
};

struct Alert {
  QString text1;
  QString text2;
  QString type;
  cereal::ControlsState::AlertSize size;
  cereal::ControlsState::AlertStatus status;
  AudibleAlert sound;

  bool equal(const Alert &a2) {
    return text1 == a2.text1 && text2 == a2.text2 && type == a2.type && sound == a2.sound;
  }

  static Alert get(const SubMaster &sm, uint64_t started_frame) {
    const cereal::ControlsState::Reader &cs = sm["controlsState"].getControlsState();
    const uint64_t controls_frame = sm.rcv_frame("controlsState");

    Alert alert = {};
    if (controls_frame >= started_frame) {  // Don't get old alert.
      alert = {cs.getAlertText1().cStr(), cs.getAlertText2().cStr(),
               cs.getAlertType().cStr(), cs.getAlertSize(),
               cs.getAlertStatus(),
               cs.getAlertSound()};
    }

    if (!sm.updated("controlsState") && (sm.frame - started_frame) > 5 * UI_FREQ) {
      const int CONTROLS_TIMEOUT = 5;
      const int controls_missing = (nanos_since_boot() - sm.rcv_time("controlsState")) / 1e9;

      // Handle controls timeout
      if (controls_frame < started_frame) {
        // car is started, but controlsState hasn't been seen at all
        alert = {"openpilot Unavailable", "Waiting for controls to start",
                 "controlsWaiting", cereal::ControlsState::AlertSize::MID,
                 cereal::ControlsState::AlertStatus::NORMAL,
                 AudibleAlert::NONE};
      } else if (controls_missing > CONTROLS_TIMEOUT && !Hardware::PC()) {
        // car is started, but controls is lagging or died
        if (cs.getEnabled() && (controls_missing - CONTROLS_TIMEOUT) < 10) {
          alert = {"TAKE CONTROL IMMEDIATELY", "Controls Unresponsive",
                   "controlsUnresponsive", cereal::ControlsState::AlertSize::FULL,
                   cereal::ControlsState::AlertStatus::CRITICAL,
                   AudibleAlert::WARNING_IMMEDIATE};
        } else {
          alert = {"Controls Unresponsive", "Reboot Device",
                   "controlsUnresponsivePermanent", cereal::ControlsState::AlertSize::MID,
                   cereal::ControlsState::AlertStatus::NORMAL,
                   AudibleAlert::NONE};
        }
      }
    }
    return alert;
  }
};

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
  STATUS_CRUISE_STOP,
} UIStatus;

enum PrimeType {
  UNKNOWN = -1,
  NONE = 0,
  MAGENTA = 1,
  LITE = 2,
  BLUE = 3,
  MAGENTA_NEW = 4,
  PURPLE = 5,
};

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] = QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = QColor(0x91, 0x9b, 0x95, 0xf1),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0xf1),
  [STATUS_WARNING] = QColor(0xDA, 0x6F, 0x25, 0xf1),
  [STATUS_ALERT] = QColor(0xC9, 0x22, 0x31, 0xf1),
  [STATUS_CRUISE_STOP] = QColor(0x00, 0x64, 0xC8, 0x96),
};

static std::map<cereal::ControlsState::AlertStatus, QColor> alert_colors = {
  {cereal::ControlsState::AlertStatus::NORMAL, QColor(0x15, 0x15, 0x15, 0xf1)},
  {cereal::ControlsState::AlertStatus::USER_PROMPT, QColor(0xDA, 0x6F, 0x25, 0xf1)},
  {cereal::ControlsState::AlertStatus::CRITICAL, QColor(0xC9, 0x22, 0x31, 0xf1)},
};

typedef struct {
  QPointF v[33 * 2];
  int cnt;
} line_vertices_data;

typedef struct {
    float x, y;
} vertex_data;

typedef struct {
    float x, y, d, v, y_rel;
} lead_vertex_data;

typedef struct UIScene {
  bool calibration_valid = false;
  bool calibration_wide_valid  = false;
  bool wide_cam = true;
  mat3 view_from_calib = DEFAULT_CALIBRATION;
  mat3 view_from_wide_calib = DEFAULT_CALIBRATION;
  cereal::PandaState::PandaType pandaType;

  // modelV2
  float lane_line_probs[4];
  float road_edge_stds[2];
  QPolygonF track_vertices;
  QPolygonF lane_line_vertices[4];
  QPolygonF road_edge_vertices[2];
  QPolygonF lane_barrier_vertices[2];
  QPointF path_end_left_vertices[2];
  QPointF path_end_right_vertices[2];
  float max_distance;

  // lead
  QPointF lead_vertices[2];
  bool lead_radar[2] = {false, false};
  std::vector<lead_vertex_data> lead_vertices_side;
  // DMoji state
  float driver_pose_vals[3];
  float driver_pose_diff[3];
  float driver_pose_sins[3];
  float driver_pose_coss[3];
  vec3 face_kpts_draw[std::size(default_face_kpts_3d)];

  bool navigate_on_openpilot = false;

  float light_sensor;
  bool started, ignition, is_metric, map_on_left, longitudinal_control;
  uint64_t started_frame;
  cereal::DeviceState::Reader deviceState;
  cereal::LongitudinalPlan::Reader longitudinal_plan;
} UIScene;

class UIState : public QObject {
  Q_OBJECT

public:
  UIState(QObject* parent = 0);
  void updateStatus();
  inline bool worldObjectsVisible() const {
    return sm->rcv_frame("liveCalibration") > scene.started_frame;
  }
  inline bool engaged() const {
    return scene.started && (*sm)["controlsState"].getControlsState().getEnabled();
  }

  void setPrimeType(PrimeType type);
  inline PrimeType primeType() const { return prime_type; }
  inline bool hasPrime() const { return prime_type != PrimeType::UNKNOWN && prime_type != PrimeType::NONE; }

  int fb_w = 0, fb_h = 0;
  NVGcontext *vg;
  std::map<std::string, int> images;

  std::unique_ptr<SubMaster> sm;

  UIStatus status;
  UIScene scene = {};

  QString language;

  QTransform car_space_transform;
  bool show_debug = false;
  int show_datetime = 1;
  bool show_tpms = true;
  int show_accel = 2;
  bool show_steer_rotate = true;
  int show_path_end = 1;
  int show_steer_mode = 0;
  bool show_device_stat = true;
  bool show_conn_info = true;
  int  show_lane_info = 2;
  bool show_blind_spot = true;
  int show_gap_info = 1;
  int show_dm_info = -1;
  int show_radar_info = 0;
  int show_mode = 1;
  float show_z_offset = 1.22;
  int show_path_mode = 0;
  int show_path_color = 14;
  int show_path_mode_cruise_off = 0;
  int show_path_color_cruise_off = 14;
  int show_path_mode_lane = 0;
  int show_path_color_lane = 14;
  float show_path_width = 1.0;
  int show_plot_mode = 0;

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void primeTypeChanged(PrimeType prime_type);

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = false;
  PrimeType prime_type = PrimeType::UNKNOWN;
};

UIState *uiState();

// device management class
class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);
  bool isAwake() { return awake; }
  void setOffroadBrightness(int brightness) {
    offroad_brightness = std::clamp(brightness, 0, 100);
  }

private:
  bool awake = false;
  int interactive_timeout = 0;
  bool ignition_on = false;

  int offroad_brightness = BACKLIGHT_OFFROAD;
  int last_brightness = 0;
  FirstOrderFilter brightness_filter;
  QFuture<void> brightness_future;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);
  void setAwake(bool on);

signals:
  void displayPowerChanged(bool on);
  void interactiveTimeout();

public slots:
  void resetInteractiveTimeout(int timeout = -1);
  void update(const UIState &s);
};

Device *device();

void ui_update_params(UIState *s);
int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height);
void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model,
                  const cereal::UiPlan::Reader &plan);
void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd);
void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line);
void update_line_data(const UIState *s, const cereal::XYZTData::Reader &line,
                      float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert);
