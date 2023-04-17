#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>

#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

//#define __TEST
#ifdef __TEST
double start_millis = 0.0;
double check_millis[10] = { 0.0, };
void set_start_millis()
{
    int i;
    start_millis = millis_since_boot();
    for (i = 0; i < 10; i++) check_millis[i] = start_millis;
}
void print_millis() {
    int i;
    for (i = 0; i < 10; i++) {
        printf("%5.2f ", check_millis[i] - start_millis);
    }
    printf("\n");
}
#endif
OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QRect rc = rect();
  if(false && isMapVisible()) {
    UIState *s = uiState();
    if(!s->scene.map_on_left)
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
    else {
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
      rc.setX((topWidget(this)->width() / 2));
    }
  }
  if(rc.contains(e->pos())) {
    QPoint endPos = e->pos();
    int dx = endPos.x() - startPos.x();
    int dy = endPos.y() - startPos.y();
    if(std::abs(dx) > 250 || std::abs(dy) > 200) {

      if(std::abs(dx) < std::abs(dy)) {

        if(dy < 0) { // upward
          Params().remove("CalibrationParams");
          Params().remove("LiveParameters");
          QTimer::singleShot(1500, []() {
            Params().putBool("SoftRestartTriggered", true);
          });

          QSound::play("../assets/sounds/reset_calibration.wav");
        }
        else { // downward
          QTimer::singleShot(500, []() {
            Params().putBool("SoftRestartTriggered", true);
          });
        }
      }
      else if(std::abs(dx) > std::abs(dy)) {
        if(dx < 0) { // right to left
          if(recorder)
            recorder->toggle();
        }
        else { // left to right
          if(recorder)
            recorder->toggle();
        }
      }

      return;
    }

    if (map != nullptr) {
      bool sidebarVisible = geometry().x() > 0;
      map->setVisible(!sidebarVisible && !map->isVisible());
    }
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {

  QRect rc = rect();
  if(false && isMapVisible()) {
    UIState *s = uiState();
    if(!s->scene.map_on_left)
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
    else {
      rc.setWidth(rc.width() - (topWidget(this)->width() / 2));
      rc.setX((topWidget(this)->width() / 2));
    }
  }

  printf("%d, %d, %d, %d\n", rc.x(), rc.y(), rc.width(), rc.height());
  if(rc.contains(e->pos())) {
    startPos = e->pos();
  }

  QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->insertWidget(0, m);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({}, bg);

  if(offroad && recorder) {
    recorder->stop(false);
  }
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  bg.setAlpha(100);
  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Inter", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Inter", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Inter", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Inter", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Inter", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

ExperimentalButton::ExperimentalButton(QWidget *parent) : QPushButton(parent) {
  setVisible(false);
  setFixedSize(btn_size, btn_size);
  setCheckable(true);

  params = Params();
  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});

  QObject::connect(this, &QPushButton::toggled, [=](bool checked) {
    params.putBool("ExperimentalMode", checked);
  });
}

void ExperimentalButton::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // button is "visible" if engageable or enabled
  const auto cs = sm["controlsState"].getControlsState();
  setVisible(cs.getEngageable() || cs.getEnabled());

  // button is "checked" if experimental mode is enabled
  setChecked(sm["controlsState"].getControlsState().getExperimentalMode());

  // disable button when experimental mode is not available, or has not been confirmed for the first time
  const auto cp = sm["carParams"].getCarParams();
  const bool experimental_mode_available = cp.getExperimentalLongitudinalAvailable() ? params.getBool("ExperimentalLongitudinalEnabled") : cp.getOpenpilotLongitudinalControl();
  setEnabled(params.getBool("ExperimentalModeConfirmed") && experimental_mode_available);
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  QPoint center(btn_size / 2, btn_size / 2);
  QPixmap img = isChecked() ? experimental_img : engage_img;

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(isDown() ? 0.8 : 1.0);
  p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
}

AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : last_update_params(0), apilot_filter_x(UI_FREQ, 1.0, 1./UI_FREQ), apilot_filter_y(UI_FREQ, 1.0, 1. / UI_FREQ), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  main_layout->setSpacing(0);

  //experimental_btn = new ExperimentalButton(this);
  //main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  ic_radar = QPixmap("../assets/images/radar_red.png");
  ic_radar_vision = QPixmap("../assets/images/radar_vision.png");
  ic_radar_no = QPixmap("../assets/images/no_radar.png");
  ic_steer_momo = QPixmap("../assets/images/steer_momo.png");
  ic_steer_red = QPixmap("../assets/images/steer_red.png");
  ic_steer_green = QPixmap("../assets/images/steer_green.png");
  ic_steer_yellow = QPixmap("../assets/images/steer_yellow.png");
  ic_lane_change_l = QPixmap("../assets/images/lane_change_l.png");
  ic_lane_change_r = QPixmap("../assets/images/lane_change_r.png");
  ic_lane_change_inhibit = QPixmap("../assets/images/lane_change_inhibit.png");
  ic_lane_change_steer = QPixmap("../assets/images/lane_change_steer.png");
  ic_bsd_l = QPixmap("../assets/images/bsd_l.png");
  ic_bsd_r = QPixmap("../assets/images/bsd_r.png");
  ic_turn_l = QPixmap("../assets/images/turn_l.png");
  ic_turn_r = QPixmap("../assets/images/turn_r.png");
  ic_blinker_l = QPixmap("../assets/images/blink_l.png");
  ic_blinker_r = QPixmap("../assets/images/blink_r.png");
  ic_speed_bg = QPixmap("../assets/images/speed_bg.png");
  ic_traffic_green = QPixmap("../assets/images/traffic_green.png");
  ic_traffic_red = QPixmap("../assets/images/traffic_red.png");
  ic_tire = QPixmap("../assets/images/img_tire.png");
  ic_road_speed = QPixmap("../assets/images/road_speed.png");


}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  //experimental_btn->updateState(s);

  const bool cs_alive = sm.alive("controlsState");
  setProperty("left_blindspot", cs_alive && sm["carState"].getCarState().getLeftBlindspot());
  setProperty("right_blindspot", cs_alive && sm["carState"].getCarState().getRightBlindspot());
  const auto cs = sm["controlsState"].getControlsState();

  // update DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    dmActive = sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode();
  }

  hideDM = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  dm_fade_state = fmax(0.0, fmin(1.0, dm_fade_state+0.2*(0.5-(float)(dmActive))));
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // show_lane_info: -1 : all off, 0: path only, 1: path+lane, 2: path+lane+edge
  // lanelines
  if (s->show_lane_info > 0) {
      for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
          painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i] * 2.0, 0.5, 1.0)));
          painter.drawPolygon(scene.lane_line_vertices[i]);
      }
  }
  if (s->show_blind_spot) {
      painter.setBrush(QColor(255, 215, 000, 150));
      if (left_blindspot) painter.drawPolygon(scene.lane_barrier_vertices[0]);
      if (right_blindspot) painter.drawPolygon(scene.lane_barrier_vertices[1]);
  }

  // road edges
  if (s->show_lane_info > 1) {
      for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
          painter.setBrush(QColor::fromRgbF(1.0, 0, 1.0, std::clamp<float>(3.0 - scene.road_edge_stds[i], 0.0, 1.0)));
          painter.drawPolygon(scene.road_edge_vertices[i]);
      }
  }

  // paint path
  if (s->show_lane_info > -1) {
      QLinearGradient bg(0, height(), 0, height() / 4);
      float start_hue, end_hue;
      if (sm["controlsState"].getControlsState().getExperimentalMode() || true) {
          const auto& acceleration = sm["modelV2"].getModelV2().getAcceleration();
          float acceleration_future = 0;
          if (acceleration.getZ().size() > 16) {
              acceleration_future = acceleration.getX()[16];  // 2.5 seconds
          }
          start_hue = 60;
          // speed up: 120, slow down: 0
          end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

          // FIXME: painter.drawPolygon can be slow if hue is not rounded
          end_hue = int(end_hue * 100 + 0.5) / 100;

          bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
          bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
          bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
      }
      else {
          bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
          bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
          bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
      }
      painter.setBrush(bg);
      painter.drawPolygon(scene.track_vertices);
  }

  painter.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int x = (btn_size - 24) / 2 + (bdr_s * 2);
  int y = rect().bottom() - footer_h / 2;
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, x, y, dm_img, blackColor(0), opacity);

  // circle background
  painter.setOpacity(1.0);
  painter.setPen(Qt::NoPen);
  painter.setBrush(blackColor(70));
  painter.drawEllipse(x - btn_size / 2, y - btn_size / 2, btn_size, btn_size);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}
void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, bool is_radar) {
  painter.save();
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  painter.restore();
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();
  const cereal::RadarState::Reader &radar_state = sm["radarState"].getRadarState();

  QPainter p(this);

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }

    p.beginNativePainting();
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
    p.endNativePainting();

  }

  if (s->worldObjectsVisible()) {
    if (sm.rcv_frame("modelV2") > s->scene.started_frame) {
      update_model(s, sm["modelV2"].getModelV2(), sm["uiPlan"].getUiPlan());
      if (sm.rcv_frame("radarState") > s->scene.started_frame) {
        update_leads(s, radar_state, sm["modelV2"].getModelV2().getPosition());
      }
    }
    switch (s->show_mode) {
    case 0: drawHud(p, model); break;
    default:drawHudApilot(p, model); break;
    }


    // DMoji
    if (s->show_dm_info==1 && !hideDM && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
      update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, false);
      drawDriverState(p, s);
    }
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  m_fps = fps;
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);
#ifdef __TEST
  printf("elapsed = %.2f\n", cur_draw_t - start_draw_t);
#endif

  auto now = millis_since_boot();
  if (now - last_update_params > 2 * 1) {
      last_update_params = now;
      ui_update_params(uiState());
  }

}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  prev_draw_t = millis_since_boot();
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
}

void AnnotatedCameraWidget::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void AnnotatedCameraWidget::drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model) {

  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);
  const cereal::RadarState::Reader &radar_state = sm["radarState"].getRadarState();

  drawLaneLines(p, s);

  
  auto lead_one = radar_state.getLeadOne();
  auto lead_two = radar_state.getLeadTwo();
  if (lead_one.getStatus()) {
    drawLead(p, lead_one, s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }
  if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
    drawLead(p, lead_two, s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }

  drawMaxSpeed(p);
  drawSpeed(p);
  if (s->show_steer_mode) drawSteer(p);
  if (s->show_device_stat) drawDeviceState(p);
  drawTurnSignals(p);
  drawGpsStatus(p);
  if (s->show_debug) drawDebugText(p);

#if 0
  const auto controls_state = sm["controlsState"].getControlsState();
  //const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto car_control = sm["carControl"].getCarControl();
  const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  const auto torque_state = controls_state.getLateralControlState().getTorqueState();

  QString infoText;
  infoText.sprintf("TP(%.2f/%.2f) LTP(%.2f/%.2f/%.0f) AO(%.2f/%.2f) SR(%.2f) SAD(%.2f) SCC(%d)",

                      torque_state.getLatAccelFactor(),
                      torque_state.getFriction(),

                      live_torque_params.getLatAccelFactorRaw(),
                      live_torque_params.getFrictionCoefficientRaw(),
                      live_torque_params.getTotalBucketPoints(),

                      live_params.getAngleOffsetDeg(),
                      live_params.getAngleOffsetAverageDeg(),

                      car_control.getSteerRatio(),
                      car_control.getSteerActuatorDelay(),

                      car_control.getSccBus()
                      );

  // info

  p.save();
  configFont(p, "Inter", 34, "Regular");
  p.setPen(QColor(0xff, 0xff, 0xff, 200));
  p.drawText(rect().left() + 20, rect().height() - 15, infoText);
  p.restore();
#endif
  drawBottomIcons(p);
}

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "  -";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

void AnnotatedCameraWidget::drawBottomIcons(QPainter &p) {
  p.save();
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  //auto car_control = sm["carControl"].getCarControl();
  auto controls_state = sm["controlsState"].getControlsState();

  // tpms
  {
    const int w = 58;
    const int h = 126;
    const int x = 110;
    const int y = height() - h - 85;

    auto tpms = car_state.getTpms();
    const float fl = tpms.getFl();
    const float fr = tpms.getFr();
    const float rl = tpms.getRl();
    const float rr = tpms.getRr();

    p.setOpacity(0.8);
    p.drawPixmap(x, y, w, h, ic_tire_pressure);

    configFont(p, "Inter", 38, "Bold");

    QFontMetrics fm(p.font());
    QRect rcFont = fm.boundingRect("9");

    int center_x = x + 3;
    int center_y = y + h/2;
    const int marginX = (int)(rcFont.width() * 2.7f);
    const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

    drawText2(p, center_x-marginX, center_y-marginY-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
    drawText2(p, center_x+marginX, center_y-marginY-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
    drawText2(p, center_x-marginX, center_y+marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
    drawText2(p, center_x+marginX, center_y+marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
  }

  int x = radius / 2 + (bdr_s * 2) + (radius + 50) * 2;
  const int y = rect().bottom() - footer_h / 2 - 10;

  // cruise gap
  int gap = controls_state.getLongCruiseGap();

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(QColor(0, 0, 0, 255 * .1f)));
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  QString str;
  float textSize = 50.f;
  QColor textColor = QColor(255, 255, 255, 200);

  if(gap <= 0) {
    str = "N/A";
  }
  else if(gap == 4) {
    str = "AUTO";
    textColor = QColor(120, 255, 120, 200);
  }
  else {
    str.sprintf("%d", (int)gap);
    textColor = QColor(120, 255, 120, 200);
    textSize = 70.f;
  }

  configFont(p, "Inter", 35, "Bold");
  drawText(p, x, y-20, "GAP", 200);

  configFont(p, "Inter", textSize, "Bold");
  drawTextWithColor(p, x, y+50, str, textColor);

  // brake
  x = radius / 2 + (bdr_s * 2) + (radius + 50) * 3;
  bool brake_valid = car_state.getBrakeLights();
  float img_alpha = brake_valid ? 1.0f : 0.15f;
  float bg_alpha = brake_valid ? 0.3f : 0.1f;
  drawIcon(p, x, y, ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);

  // auto hold
  int autohold = car_state.getBrakeHoldActive();
  if(autohold >= 0) {
    x = radius / 2 + (bdr_s * 2) + (radius + 50) * 4;
    img_alpha = autohold > 0 ? 1.0f : 0.15f;
    bg_alpha = autohold > 0 ? 0.3f : 0.1f;
    drawIcon(p, x, y, autohold > 1 ? ic_autohold_warning : ic_autohold_active,
            QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  }

  p.restore();
}

void AnnotatedCameraWidget::drawSpeed(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgoCluster() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 230);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230);
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Inter", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 230, speed, color);

  configFont(p, "Inter", 66, "Regular");
  drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200);

  p.restore();
}

QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

void AnnotatedCameraWidget::drawMaxSpeed(QPainter &p) {
  p.save();

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  //const auto car_control = sm["carControl"].getCarControl();
  //const auto car_state = sm["carState"].getCarState();
  const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  bool is_metric = s->scene.is_metric;

  // kph
  const auto cs = sm["controlsState"].getControlsState();
  float applyMaxSpeed = cs.getVCruiseOut(); //car_control.getApplyMaxSpeed();
  float cruiseMaxSpeed = cs.getVCruiseCluster(); //car_control.getCruiseMaxSpeed();

  bool is_cruise_set = (cs.getLongActiveUser()>0); //cruiseState.getEnabled();

  int activeNDA = road_limit_speed.getActive();
  int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
  int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
  int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
  int sectionLeftDist = road_limit_speed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0) {
      int w = 120;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      int y = 40 - bdr_s;

      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, ic_nda);
  }


  const int x_start = 30;
  const int y_start = 30;

  int board_width = 210;
  int board_height = 384;

  const int corner_radius = 32;
  int max_speed_height = 210;

  QColor bgColor = QColor(0, 0, 0, 166);

  {
    // draw board
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);

    if(limit_speed > 0) {
      board_width = limit_speed < 100 ? 210 : 230;
      board_height = max_speed_height + board_width;

      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height-board_width/2), corner_radius, corner_radius);
      path.addRoundedRect(QRectF(x_start, y_start+corner_radius, board_width, board_height-corner_radius), board_width/2, board_width/2);
    }
    else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
      board_height = 485;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }
    else {
      max_speed_height = 235;
      board_height = max_speed_height;
      path.addRoundedRect(QRectF(x_start, y_start, board_width, board_height), corner_radius, corner_radius);
    }

    p.setPen(Qt::NoPen);
    p.fillPath(path.simplified(), bgColor);
  }

  QString str;

  // Max Speed
  {
    p.setPen(QColor(255, 255, 255, 230));

    if(is_cruise_set) {
      configFont(p, "Inter", 80, "Bold");

      if(is_metric)
        str.sprintf( "%d", (int)(cruiseMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      configFont(p, "Inter", 60, "Bold");
      str = "N/A";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 35);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }


  // applyMaxSpeed
  {
    p.setPen(QColor(255, 255, 255, 180));

    configFont(p, "Inter", 50, "Bold");
    if(is_cruise_set && applyMaxSpeed > 0) {
      if(is_metric)
        str.sprintf( "%d", (int)(applyMaxSpeed + 0.5));
      else
        str.sprintf( "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));
    }
    else {
      str = "MAX";
    }

    QRect speed_rect = getRect(p, Qt::AlignCenter, str);
    QRect max_speed_rect(x_start, y_start + max_speed_height/2, board_width, max_speed_height/2);
    speed_rect.moveCenter({max_speed_rect.center().x(), 0});
    speed_rect.moveTop(max_speed_rect.top() + 24);
    p.drawText(speed_rect, Qt::AlignCenter | Qt::AlignVCenter, str);
  }

  //
  if(limit_speed > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = 18;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, 25));
    p.drawEllipse(board_rect);

    p.setPen(QPen(Qt::black, padding));

    str.sprintf("%d", limit_speed);
    configFont(p, "Inter", 70, "Bold");

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    if(left_dist > 0) {
      // left dist
      QRect rcLeftDist;
      QString strLeftDist;

      if(left_dist < 1000)
        strLeftDist.sprintf("%dm", left_dist);
      else
        strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

      QFont font("Inter");
      font.setPixelSize(55);
      font.setStyleName("Bold");

      QFontMetrics fm(font);
      int width = fm.width(strLeftDist);

      padding = 10;

      int center_x = x_start + board_width / 2;
      rcLeftDist.setRect(center_x - width / 2, y_start+board_height+15, width, font.pixelSize()+10);
      rcLeftDist.adjust(-padding*2, -padding, padding*2, padding);

      p.setPen(Qt::NoPen);
      p.setBrush(bgColor);
      p.drawRoundedRect(rcLeftDist, 20, 20);

      configFont(p, "Inter", 55, "Bold");
      p.setBrush(Qt::NoBrush);
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(rcLeftDist, Qt::AlignCenter|Qt::AlignVCenter, strLeftDist);
    }
  }
  else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      configFont(p, "Inter", 35, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimitSpeed);
      configFont(p, "Inter", 75, "Bold");

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
  }

  p.restore();
}

void AnnotatedCameraWidget::drawSteer(QPainter &p) {
  p.save();

  int x = 30;
  int y = 540;

  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  configFont(p, "Inter", 50, "Bold");

  QString str;
  int width = 192;

  str.sprintf("%.0f°", steer_angle);
  QRect rect = QRect(x, y, width, width);

  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  str.sprintf("%.0f°", desire_angle);
  rect.setRect(x, y + 80, width, width);

  p.setPen(QColor(155, 255, 155, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  p.restore();
}

template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
  std::vector<T> xData(x_list);
  std::vector<T> yData(y_list);
  int size = xData.size();

  int i = 0;
  if(x >= xData[size - 2]) {
    i = size - 2;
  }
  else {
    while ( x > xData[i+1] ) i++;
  }
  T xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
  if (!extrapolate) {
    if ( x < xL ) yR = yL;
    if ( x > xR ) yL = yR;
  }

  T dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}

void AnnotatedCameraWidget::drawDeviceState(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto deviceState = sm["deviceState"].getDeviceState();

  const auto freeSpacePercent = deviceState.getFreeSpacePercent();

  const auto cpuTempC = deviceState.getCpuTempC();
  //const auto gpuTempC = deviceState.getGpuTempC();
  float ambientTemp = deviceState.getAmbientTempC();

  float cpuTemp = 0.f;
  //float gpuTemp = 0.f;

  if(std::size(cpuTempC) > 0) {
    for(int i = 0; i < std::size(cpuTempC); i++) {
      cpuTemp += cpuTempC[i];
    }
    cpuTemp = cpuTemp / (float)std::size(cpuTempC);
  }

  /*if(std::size(gpuTempC) > 0) {
    for(int i = 0; i < std::size(gpuTempC); i++) {
      gpuTemp += gpuTempC[i];
    }
    gpuTemp = gpuTemp / (float)std::size(gpuTempC);
    cpuTemp = (cpuTemp + gpuTemp) / 2.f;
  }*/

  if (uiState()->show_mode) {
      QString str;
      auto car_state = sm["carState"].getCarState();
      str.sprintf("STORAGE: %.0f%%   CPU: %.0f°C    AMBIENT: %.0f°C", freeSpacePercent, cpuTemp, ambientTemp);
      int r = interp<float>(cpuTemp, { 50.f, 90.f }, { 200.f, 255.f }, false);
      int g = interp<float>(cpuTemp, { 50.f, 90.f }, { 255.f, 200.f }, false);
      QColor textColor = QColor(r, g, 200, 200);
      configFont(p, "Inter", 30, "Bold");
      if (width() > 1200) {
          drawTextWithColor(p, width() - 350, 35, str, textColor);
          float engineRpm = car_state.getEngineRpm();
          float motorRpm = car_state.getMotorRpm();
          str.sprintf("FPS: %d, %s: %.0f CHARGE: %.0f%%", m_fps, (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
          drawTextWithColor(p, width() - 350, 80, str, textColor);
      }
  }
  else {
      int w = 192;
      int x = width() - (30 + w);
      int y = 340;

      QString str;
      QRect rect;

      configFont(p, "Inter", 50, "Bold");
      str.sprintf("%.0f%%", freeSpacePercent);
      rect = QRect(x, y, w, w);

      int r = interp<float>(freeSpacePercent, { 10.f, 90.f }, { 255.f, 200.f }, false);
      int g = interp<float>(freeSpacePercent, { 10.f, 90.f }, { 200.f, 255.f }, false);
      p.setPen(QColor(r, g, 200, 200));
      p.drawText(rect, Qt::AlignCenter, str);

      y += 55;
      configFont(p, "Inter", 25, "Bold");
      rect = QRect(x, y, w, w);
      p.setPen(QColor(255, 255, 255, 200));
      p.drawText(rect, Qt::AlignCenter, "STORAGE");

      y += 80;
      configFont(p, "Inter", 50, "Bold");
      str.sprintf("%.0f°C", cpuTemp);
      rect = QRect(x, y, w, w);
      r = interp<float>(cpuTemp, { 50.f, 90.f }, { 200.f, 255.f }, false);
      g = interp<float>(cpuTemp, { 50.f, 90.f }, { 255.f, 200.f }, false);
      p.setPen(QColor(r, g, 200, 200));
      p.drawText(rect, Qt::AlignCenter, str);

      y += 55;
      configFont(p, "Inter", 25, "Bold");
      rect = QRect(x, y, w, w);
      p.setPen(QColor(255, 255, 255, 200));
      p.drawText(rect, Qt::AlignCenter, "CPU");

      y += 80;
      configFont(p, "Inter", 50, "Bold");
      str.sprintf("%.0f°C", ambientTemp);
      rect = QRect(x, y, w, w);
      r = interp<float>(ambientTemp, { 35.f, 60.f }, { 200.f, 255.f }, false);
      g = interp<float>(ambientTemp, { 35.f, 60.f }, { 255.f, 200.f }, false);
      p.setPen(QColor(r, g, 200, 200));
      p.drawText(rect, Qt::AlignCenter, str);

      y += 55;
      configFont(p, "Inter", 25, "Bold");
      rect = QRect(x, y, w, w);
      p.setPen(QColor(255, 255, 255, 200));
      p.drawText(rect, Qt::AlignCenter, "AMBIENT");
  }

  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  p.save();

  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.restore();
}

void AnnotatedCameraWidget::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 120;
  int h = 100;
  int x = width() - w - 30;
  int y = 30;

  p.save();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Inter", 40, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);

  p.restore();
}

void AnnotatedCameraWidget::drawDebugText(QPainter &p) {

  p.save();

  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 150;

  const int text_x = width()/2 + 220;
  const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();

  configFont(p, "Inter", 40, "Regular");
  p.setPen(QColor(255, 255, 255, 200));

  str.sprintf("LT[%.0f]:%s (%.3f/%.3f)", live_torque_params.getTotalBucketPoints(), live_torque_params.getLiveValid() ? "ON" : "OFF", live_torque_params.getLatAccelFactorFiltered(), live_torque_params.getFrictionCoefficientFiltered());
  p.drawText(text_x, y, str);
  p.drawText(text_x, y+80, QString::fromStdString(live_torque_params.getDebugText().cStr()));

  auto controls_state = sm["controlsState"].getControlsState();
  p.drawText(text_x, y + 160, QString::fromStdString(controls_state.getDebugText2().cStr()));
  p.drawText(text_x, y + 240, QString::fromStdString(controls_state.getDebugText1().cStr()));

  p.restore();
  /*p.save();
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80;
  const int height = 60;

  const int text_x = width()/2 + 250;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  float applyAccel = controls_state.getApplyAccel();

  float aReqValue = controls_state.getAReqValue();
  float aReqValueMin = controls_state.getAReqValueMin();
  float aReqValueMax = controls_state.getAReqValueMax();

  float vEgo = car_state.getVEgo();
  float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  float vPid = controls_state.getVPid();
  float upAccelCmd = controls_state.getUpAccelCmd();
  float uiAccelCmd = controls_state.getUiAccelCmd();
  float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

  configFont(p, "Inter", 35, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Accel: %.3f\n", accel);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("aEgo: %.3f, %.3f\n", car_state.getAEgo(), car_state.getABasis());
  p.drawText(text_x, y, str);

  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  y += height;
  str.sprintf("Lead: %.1f/%.1f/%.1f\n", radar_dist, vision_dist, (radar_dist - vision_dist));
  p.drawText(text_x, y, str);

  p.restore();*/
}

void AnnotatedCameraWidget::drawLeadApilot(QPainter& painter, const cereal::ModelDataV2::Reader& model) {

    UIState* s = uiState();
    const UIScene& scene = s->scene;
    SubMaster& sm = *(s->sm);
    auto leads = model.getLeadsV3();
    const cereal::ModelDataV2::LeadDataV3::Reader& lead_data = leads[0];
    const QPointF& vd = s->scene.lead_vertices[0];
    //bool is_radar = s->scene.lead_radar[0];
    bool no_radar = leads[0].getProb() < .5;
    bool    uiDrawSteeringRotate = s->show_steer_rotate;
    bool    uiDrawPathEnd = s->show_path_end;   // path끝에 표시를 넣을건지..

#ifndef __TEST
    if (!sm.alive("controlsState") || !sm.alive("radarState") || !sm.alive("carControl")) return;
#endif

    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
    auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];
    auto controls_state = sm["controlsState"].getControlsState();
    auto car_control = sm["carControl"].getCarControl();
    auto car_state = sm["carState"].getCarState();
    int longActiveUser = controls_state.getLongActiveUser();
    int longActiveUserReady = controls_state.getLongActiveUserReady();

    int     uiDrawSeq = sm.frame % 3;   // 시간이 많이 걸리는것들은 해당순서에서 그리자,, 0,1,2

    // Path의 끝위치를 계산 및 표시
    int     track_vertices_len = scene.track_vertices.length();
    float path_x = width() / 2;
    float path_y = height() - 400;
    float path_width = 160;
    float path_bx = path_x;
    //float path_by = path_y;
    //float path_bwidth = path_width;
    {
        if (track_vertices_len >= 10) {
            path_width = scene.track_vertices[track_vertices_len / 2].x() - scene.track_vertices[track_vertices_len / 2 - 1].x();
            path_x = (scene.track_vertices[track_vertices_len / 2].x() + scene.track_vertices[track_vertices_len / 2 - 1].x()) / 2.;
            path_y = scene.track_vertices[track_vertices_len / 2].y();
            //path_bwidth = scene.track_vertices[0].x() - scene.track_vertices[track_vertices_len -1].x();
            path_bx = (scene.track_vertices[0].x() + scene.track_vertices[track_vertices_len - 1].x()) / 2.;
            //path_by = scene.track_vertices[0].y();
            if (uiDrawPathEnd) {
#if 1
                painter.setPen(QPen(Qt::red, 10));
                painter.drawLine(path_x - path_width / 2., path_y, path_x + path_width / 2., path_y);
#else
                QRect rectPath(path_x - path_width / 2., path_y - 5, path_width, 5);
                QRect rectPathL(path_x - path_width / 2., path_y - 5, 10, 10);
                QRect rectPathR(path_x + path_width / 2. - 5, path_y - 5, 10, 10);
                painter.setPen(Qt::NoPen);
                painter.setBrush(redColor(160));
                painter.drawRect(rectPath);
                painter.drawRect(rectPathL);
                painter.drawRect(rectPathR);
#endif
            }
        }
    }

    // 과녁을 표시할 위치를 계산
    const int icon_size = 256;
    const float d_rel = lead_data.getX()[0];
    float x = std::clamp((float)vd.x(), 550.f, width() - 550.f);
    float y = std::clamp((float)vd.y(), 300.f, height() - 180.f);

    y -= ((icon_size / 2) - d_rel);
    if (no_radar) {
        //x = path_x;
        x = std::clamp(path_x, 300.f, width() - 300.f);
        y = path_y; // height() - 250;
    }
    if (y > height() - 400) y = height() - 400;

    if (s->show_mode == 2) {
        y = height() - 400;
        x = path_bx;
    }

    x = apilot_filter_x.update(x);
    y = apilot_filter_y.update(y);

    // 신호등(traffic)그리기.
    // 신호등내부에는 레이더거리, 비젼거리, 정지거리, 신호대기 표시함.
    int circle_size = 160;
    QColor bgColor = QColor(0, 0, 0, 166);
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    float stop_dist = 0;
    bool stopping = false;
    auto hud_control = car_control.getHudControl();
    bool radar_detected = lead_radar.getStatus() && lead_radar.getRadar();
    float radar_dist = radar_detected ? lead_radar.getDRel() : 0;
    float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 0) : 0;
    float disp_dist = (radar_detected) ? radar_dist : vision_dist;
    int brake_hold = car_state.getBrakeHoldActive();
    int soft_hold = (hud_control.getSoftHold()) ? 1 : 0;

    float v_ego = sm["carState"].getCarState().getVEgoCluster();
    float v_ego_kph = v_ego * MS_TO_KPH;
    float cur_speed = v_ego * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    bool brake_valid = car_state.getBrakeLights();
    bool bsd_l = car_state.getLeftBlindspot();
    bool bsd_r = car_state.getRightBlindspot();
    // 차로변경/자동턴 표시
    bool showDistInfo = true;
    bool showBg = (disp_dist>0.0) ? true: false;
    auto lateralPlan = sm["lateralPlan"].getLateralPlan();
    auto desire = lateralPlan.getDesire();
    auto laneChangeDirection = lateralPlan.getLaneChangeDirection();
    auto desireEvent = lateralPlan.getDesireEvent();
    float desireStateTurnLeft = (desire == cereal::LateralPlan::Desire::TURN_LEFT) ? 1 : 0;
    float desireStateTurnRight = (desire == cereal::LateralPlan::Desire::TURN_RIGHT) ? 1 : 0;
    float desireStateLaneChangeLeft = (desire == cereal::LateralPlan::Desire::LANE_CHANGE_LEFT) ? 1 : 0;
    float desireStateLaneChangeRight = (desire == cereal::LateralPlan::Desire::LANE_CHANGE_RIGHT) ? 1 : 0;

    if (desire == cereal::LateralPlan::Desire::NONE) {
        auto meta = sm["modelV2"].getModelV2().getMeta();
        desireStateTurnLeft = meta.getDesireState()[1];
        desireStateTurnRight = meta.getDesireState()[2];
        desireStateLaneChangeLeft = meta.getDesireState()[3];
        desireStateLaneChangeRight = meta.getDesireState()[4];
    }
    bool leftBlinker = car_state.getLeftBlinker();
    bool rightBlinker = car_state.getRightBlinker();
    static int blinkerTimer = 0;
    blinkerTimer = (blinkerTimer + 1) % 16;
    bool blinkerOn = (blinkerTimer <= 16 / 2);

#ifdef __TEST
    static int _desire = 0.;
    if (_desire++ > 200) _desire = 0;
    if (_desire < 50) desireStateTurnLeft = 1.0;
    else if (_desire < 100) desireStateTurnRight = 1.0;
    else if (_desire < 150) desireStateLaneChangeLeft = 1.0;
    else desireStateLaneChangeRight = 1.0;
#endif
    if ((desireStateTurnLeft>0.5) || (desireStateTurnRight>0.5) || (desireStateLaneChangeLeft>0.5) || (desireStateLaneChangeRight > 0.5)) {
        showBg = true;
        showDistInfo = false;
    }
    if(true) {
        int trafficMode = 0;
        if (longActiveUser <= 0) trafficMode = 0;  // 크루즈가 꺼져있으면... 신호등을 모두 꺼버려?
        else if (lp.getTrafficState() >= 100) trafficMode = 3; // yellow
        else {
            switch (lp.getTrafficState() % 100) {
            case 0: trafficMode = 0; break;
            case 1: trafficMode = 1;    // red
                stop_dist = lp.getXStop();
                stopping = true;
                if(s->show_mode == 2) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_traffic_red);
                showBg = true;
                break;
            case 2: trafficMode = 2; 
                if (s->show_mode == 2) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_traffic_green);
                break; // green // 표시안함.
            case 3: trafficMode = 3; showBg = true; break; // yellow
            }
        }
#ifdef __TEST
        static int traffic = 0;
        if (traffic++ > 200) traffic = 0;
        if (traffic < 50) trafficMode = 0;
        else if (traffic < 100) trafficMode = 1;
        else if (traffic < 150) trafficMode = 2;
        else trafficMode = 3;

        if (traffic < 100) { leftBlinker = true; }
        else { rightBlinker = true; }
#endif
        // steer handle 그리기..
        float steer_angle = car_state.getSteeringAngleDeg();
        static QPixmap img2 = ic_steer_momo;
#ifdef __TEST
        static float steer_ang = 0.0;
        steer_ang += 1.0;
        steer_angle = steer_ang;
#endif
        if (s->show_steer_mode == 1) {
            painter.setOpacity(0.7);
            if (uiDrawSteeringRotate) {      // 시간이 많이(3msec)걸려 3번에 한번씩만 그리자..
                if (uiDrawSeq == 0) {
                    switch (trafficMode) {
                    case 0: img2 = ic_steer_momo.transformed(QTransform().rotate(-steer_angle)); break;
                    case 1: img2 = ic_steer_red.transformed(QTransform().rotate(-steer_angle)); break;
                    case 2: img2 = ic_steer_green.transformed(QTransform().rotate(-steer_angle)); break;
                    case 3: img2 = ic_steer_yellow.transformed(QTransform().rotate(-steer_angle)); break;
                    }
                    painter.drawPixmap(x - img2.width() / 2., y - img2.height() / 2., img2);
                }
                else painter.drawPixmap(x - img2.width() / 2., y - img2.height() / 2., img2);
            }
            else painter.drawPixmap(x - ic_steer_momo.width() / 2., y - ic_steer_momo.height() / 2., ic_steer_momo);
            bgColor = blackColor(160);
        }
        else if(s->show_steer_mode == 0) {
            painter.setOpacity(0.7);
            if (uiDrawSteeringRotate) {      // 시간이 많이(3msec)걸려 3번에 한번씩만 그리자..
                if (uiDrawSeq == 0) {
                    img2 = ic_steer_momo.transformed(QTransform().rotate(-steer_angle));
                    painter.drawPixmap(x - img2.width() / 2., y - img2.height() / 2., img2);
                }
                else painter.drawPixmap(x - img2.width() / 2., y - img2.height() / 2., img2);
            }
            else painter.drawPixmap(x - ic_steer_momo.width() / 2., y - ic_steer_momo.height() / 2., ic_steer_momo);
            switch (trafficMode) {
            case 0: bgColor = blackColor(90); break;
            case 1: bgColor = redColor(160); break;
            case 2: bgColor = greenColor(160); break;
            case 3: bgColor = yellowColor(160); break;
            }
        }
        else {
            showBg = false;
            /*
            switch (trafficMode) {
            case 0: bgColor = blackColor(90); break;
            case 1: bgColor = redColor(160); break;
            case 2: bgColor = greenColor(160); break;
            case 3: bgColor = yellowColor(160); break;
            }
            */
        }
        if (showBg) {
            painter.setOpacity(1.0);
            painter.setPen(Qt::NoPen);
            painter.setBrush(bgColor);
            painter.drawEllipse(x - circle_size / 2, y - circle_size / 2, circle_size, circle_size);
        }


        // 차로변경, 턴 표시~
        if (true) {
            painter.setOpacity(1.0);
            if (desireStateTurnLeft > 0.5) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_turn_l);
            else if (desireStateTurnRight > 0.5) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_turn_r);
            else if (desireStateLaneChangeLeft > 0.5) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_l);
            else if (desireStateLaneChangeRight > 0.5) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_r);
            if (desireEvent == 57) {
                painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_steer);
                painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_l);
            }
            else if (desireEvent == 58) {
                painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_steer);
                painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_r);
            }
            else if (desireEvent == 71) {
                if (laneChangeDirection == cereal::LateralPlan::LaneChangeDirection::LEFT) {
                    painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_inhibit);
                    painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_l);
                }
                else if (laneChangeDirection == cereal::LateralPlan::LaneChangeDirection::RIGHT) {
                    painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_inhibit);
                    painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_lane_change_r);
                }
            }

        }
        // blinker 표시~~
        if (true) {
            painter.setOpacity(1.0);
            if (rightBlinker && blinkerOn) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_blinker_r);
            if (leftBlinker && blinkerOn) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_blinker_l);
        }
        // BSD 표시
        if (true) {
            painter.setOpacity(1.0);
            if (bsd_l) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_bsd_l);
            if (bsd_r) painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, ic_bsd_r);
        }


#ifdef __TEST
        radar_detected = true;
        disp_dist = 127.0;
        stop_dist = 112.0;
        stopping = true;
#endif

        QString str;
        //str.sprintf("%.1fm", radar_detected ? radar_dist : vision_dist);
        QColor textColor = QColor(255, 255, 255, 255);
        //configFont(painter, "Inter", 75, "Bold");
        //drawTextWithColor(painter, x, y + sz / 1.5f + 80.0, str, textColor);

        if (radar_detected) {
            float radar_rel_speed = lead_radar.getVRel();
#ifdef __TEST
            radar_rel_speed = 20.0;
#endif
            str.sprintf("%.0f km/h", cur_speed + radar_rel_speed * 3.6);
            if (radar_rel_speed < -0.1) bgColor = redColor(200);
            else bgColor = greenColor(200);

            configFont(painter, "Inter", 40, "Bold");
            if (s->show_steer_mode >= 2) {
                int radar_y = (path_y > height() - 550) ? height() - 550 : path_y - 40;
                QRect rectRadar(path_x - 250 / 2, radar_y - 35, 250, 45);
                painter.setPen(Qt::NoPen);
                painter.setBrush(bgColor);
                painter.drawRoundedRect(rectRadar, 15, 15);
                configFont(painter, "Inter", 40, "Bold");
                textColor = whiteColor(255);
                drawTextWithColor(painter, path_x, radar_y, str, textColor);
            }
            else {
                int radar_y = y - 140;
                QRect rectRadar(x - 250 / 2, radar_y - 35, 250, 45);
                painter.setPen(Qt::NoPen);
                painter.setBrush(bgColor);
                painter.drawRoundedRect(rectRadar, 15, 15);
                configFont(painter, "Inter", 40, "Bold");
                textColor = whiteColor(255);
                drawTextWithColor(painter, x, radar_y, str, textColor);
            }
        }
        painter.setOpacity(0.7);
        painter.drawPixmap(x - icon_size / 2, y - icon_size / 2, icon_size, icon_size, (no_radar) ? ic_radar_no : (radar_detected) ? ic_radar : ic_radar_vision);

        if (no_radar) {
            if (stop_dist > 0.5 && stopping) {
                textColor = QColor(255, 255, 255, 255);
                configFont(painter, "Inter", 45, "Bold");
                if (stop_dist < 10.0) str.sprintf("%.1f", stop_dist);
                else str.sprintf("%.0f", stop_dist);
                drawTextWithColor(painter, x, y + 120.0, str, textColor);
            }
            else if (longActiveUser > 0 && (stopping|| lp.getTrafficState() >= 1000)) {
                textColor = QColor(255, 255, 255, 255);
                configFont(painter, "Inter", 40, "Bold");
                if (brake_hold || soft_hold) {
                    //drawTextWithColor(painter, x, y +120, (brake_hold) ? "AUTOHOLD" : "SOFTHOLD", textColor);
                }
                else {
                    drawTextWithColor(painter, x, y +120, (lp.getTrafficState()>=1000)?"신호오류":"신호대기", textColor);
                }
            }
        }
        else if(disp_dist>0.0) {
            textColor = QColor(255, 255, 255, 255);
            configFont(painter, "Inter", 45, "Bold");
            if (disp_dist < 10.0) str.sprintf("%.1f", disp_dist);
            else str.sprintf("%.0f", disp_dist);
            drawTextWithColor(painter, x, y + 120.0, str, textColor);
        }
    }
    // 타겟좌측 : 갭표시
    int myDrivingMode = controls_state.getMyDrivingMode();
    //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    float gap = lp.getCruiseGap();
    //float tFollow = lp.getTFollow();
    int gap1 = controls_state.getLongCruiseGap(); // car_state.getCruiseGap();
#ifdef __TEST
    myDrivingMode = 3;
#endif
    QString strDrivingMode;
    switch (myDrivingMode)
    {
    case 0: strDrivingMode = "GAP"; break;
    case 1: strDrivingMode = "연비"; break;
    case 2: strDrivingMode = "안전"; break;
    case 3: strDrivingMode = "일반"; break;
    case 4: strDrivingMode = "고속"; break;
    }
    configFont(painter, "Inter", 30, "Bold");
    QColor textColor = whiteColor(255);

    float dxGap = -128 - 10 - 40;
    drawTextWithColor(painter, x + dxGap + 15, y + 120, strDrivingMode, textColor);
    dxGap -= 60;
    if (s->show_gap_info) {
#ifdef __TEST
        static int _gap = 0;
        _gap += 10;
        if (_gap > 400) _gap = 0;
        else if (_gap < 100) gap = 1;
        else if (_gap < 200) gap = 2;
        else if (_gap < 300) gap = 3;
        else gap = 4;
#endif
        QRect rectGap(x + dxGap, y, 40, 128);
        //painter.setPen(Qt::NoPen);
        painter.setPen(QPen(Qt::white, 2));
        painter.setBrush(blackColor(150));
        rectGap = QRect(x + dxGap, y + 5, 40, 64 / 4.);
        painter.drawRect(rectGap);
        rectGap = QRect(x + dxGap, y + 5 + 64 * 1 / 4., 40, 64 / 4.);
        painter.drawRect(rectGap);
        rectGap = QRect(x + dxGap, y + 5 + 64 * 2 / 4., 40, 64 / 4.);
        painter.drawRect(rectGap);
        rectGap = QRect(x + dxGap, y + 5 + 64 * 3 / 4., 40, 64 / 4.);
        painter.drawRect(rectGap);       
        QRect rectGapPos(x + dxGap, y + 5 + 64, 40, -std::clamp((float)gap, 0.0f, 4.0f) / 4. * 64);
        painter.setBrush(greenColor(255));
        painter.drawRect(rectGapPos);
        textColor = whiteColor(255);
        configFont(painter, "Inter", 25, "Bold");
        drawTextWithColor(painter, x + dxGap + 20, y, "GAP", textColor);

#if 0
        QRect rectGap1(x1, y1, 60, 20);
        QRect rectGap2(x1, y1 + 35, 60, 20);
        QRect rectGap3(x1, y1 + 70, 60, 20);
        painter.setBrush(whiteColor(255));
        painter.drawRect(rectGap1);
        if (gap >= 2) painter.drawRect(rectGap2);
        if (gap >= 3) painter.drawRect(rectGap3);
#endif
        //configFont(painter, "Inter", 60, "Bold");
        //textColor = whiteColor(255);
        //QString str;
        //str.sprintf("%d", gap1);
        //drawTextWithColor(painter, x + dxGap + 20, y + 0, str, textColor);
    }
    // 갭정보표시 중앙위
    if(true) {
        configFont(painter, "Inter", 50, "Bold");
        textColor = whiteColor(255);
        QString str;
        //str.sprintf("%.1f", tFollow);
        str.sprintf("%d", gap1);
        drawTextWithColor(painter, x + dxGap + 15 + 60, y + 60, str, textColor);

    }

    // 타겟하단: 롱컨상태표시
    {
        QString str;
        auto xState = lp.getXState();
        if (brake_hold) str.sprintf("AUTOHOLD");
        else if (longActiveUser > 0) {
            if (xState == cereal::LongitudinalPlan::XState::E2E_STOP) str.sprintf("신호감지");
            else if (xState == cereal::LongitudinalPlan::XState::SOFT_HOLD) str.sprintf("SOFTHOLD");
            else if (xState == cereal::LongitudinalPlan::XState::LEAD) str.sprintf("LEAD");
            else if (xState == cereal::LongitudinalPlan::XState::E2E_CRUISE) str.sprintf((v_ego_kph<80)?"E2E주행":"정속주행");
            else if (xState == cereal::LongitudinalPlan::XState::CRUISE) str.sprintf("정속주행");
            else str.sprintf("UNKNOWN");
        }
        else {
            if (longActiveUserReady > 0) {
                if (xState == cereal::LongitudinalPlan::XState::SOFT_HOLD) str.sprintf("SOFTHOLD");
                else str.sprintf("크루즈대기");
            }
            else str.sprintf("수동운전");
        }
        QRect rectBrake(x - 250 / 2, y + 140, 250, 45);
        painter.setPen(Qt::NoPen);
        painter.setBrush((brake_valid) ? redColor(200) : greenColor(200));
        //painter.drawRect(rectBrake);
        painter.drawRoundedRect(rectBrake, 15, 15);
        configFont(painter, "Inter", 40, "Bold");
        textColor = whiteColor(255);
        drawTextWithColor(painter, x - 0, y + 175, str, textColor);
    }

    // Accel표시
    float accel = car_state.getAEgo();
    float dx = 128 + 10;
#ifdef __TEST
    static float accel1 = 0.0;
    accel1 += 0.2;
    if (accel1 > 2.5) accel1 = -2.5;
    accel = accel1;
#endif
    if(s->show_accel>0) {
        QRect rectAccel(x + dx, y + 5, 40, 128);
        //painter.setPen(Qt::NoPen);
        painter.setPen(QPen(Qt::white, 2));
        painter.setBrush(blackColor(150));
        painter.drawRect(rectAccel);
        QRect rectAccelPos(x + dx, y + 64 + 5, 40, -std::clamp((float)accel, -2.0f, 2.0f) / 2. * 64);
        painter.setBrush((accel >= 0.0) ? yellowColor(255) : redColor(255));
        painter.drawRect(rectAccelPos);
        textColor = whiteColor(255);
        configFont(painter, "Inter", 25, "Bold");
        drawTextWithColor(painter, x + dx + 20, y + 160, "ACC", textColor);

    }

    // RPM표시
    float engineRpm = car_state.getEngineRpm();
    float motorRpm = car_state.getMotorRpm();
#ifdef __TEST
    static float engineRpm1 = 0.0;
    engineRpm1 += 100.0;
    if (engineRpm1 > 4000.0) engineRpm1 = 0.0;
    motorRpm = engineRpm1;
#endif
    if(s->show_accel>1) {
        dx += 60;
        //str.sprintf("%s: %.0f CHARGE: %.0f%%", (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
        //drawTextWithColor(p, width() - 350, 80, str, textColor);
        //painter.setPen(Qt::NoPen);
        QRect rectRpm(x + dx, y + 5, 40, 128);
        painter.setPen(QPen(Qt::white, 2));
        painter.setBrush(blackColor(150));
        painter.drawRect(rectRpm);
        QRect rectRpmPos;
        //painter.setPen(Qt::NoPen);
        if (engineRpm > 0.0) {
            painter.setBrush(QColor(0, 0, 255, 255));
            rectRpmPos = QRect(x + dx, y + 128 + 5, 40, -std::clamp((float)engineRpm, 0.0f, 4000.0f) / 4000. * 128.0);
        }
        else {
            painter.setBrush(greenColor(255));
            rectRpmPos = QRect(x + dx, y + 128 + 5, 40, -std::clamp((float)motorRpm, 0.0f, 4000.0f) / 4000. * 128.0);
        }
        textColor = whiteColor(255);
        configFont(painter, "Inter", 25, "Bold");
        drawTextWithColor(painter, x + dx + 20, y + 160, "RPM", textColor);
        painter.drawRect(rectRpmPos);

    }


    // 속도표시
    if(true) {
        const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
        const auto car_params = sm["carParams"].getCarParams();

        //bool is_metric = s->scene.is_metric;
        //bool long_control = 1;// scc_smoother.getLongControl();

        // kph
        float applyMaxSpeed = controls_state.getVCruiseOut();// scc_smoother.getApplyMaxSpeed();
        float cruiseMaxSpeed = controls_state.getVCruiseCluster();// scc_smoother.getCruiseMaxSpeed();
        float curveSpeed = controls_state.getCurveSpeed();
        //float xCruiseTarget = lp.getXCruiseTarget() * 3.6;

        //bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);
        //bool is_cruise_set = (applyMaxSpeed > 0 && applyMaxSpeed < 255);
        //int longActiveUser = controls_state.getLongActiveUser();
        int longOverride = car_control.getLongOverride();

        int sccBus = (int)car_params.getSccBus();

        int enabled = controls_state.getEnabled();

        int activeNDA = road_limit_speed.getActive();
        int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
        int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
        int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
        int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
        int sectionLeftDist = road_limit_speed.getSectionLeftDist();

        int limit_speed = 0;
        int left_dist = 0;

        if (camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
            limit_speed = camLimitSpeed;
            left_dist = camLimitSpeedLeftDist;
        }
        else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
            limit_speed = sectionLimitSpeed;
            left_dist = sectionLeftDist;
        }

        int radar_tracks = Params().getBool("EnableRadarTracks");
        //QString nda_mode_str = QString::fromStdString(Params().get("AutoNaviSpeedCtrl"));
        //int nda_mode = nda_mode_str.toInt();

        QString top_str;
        top_str.sprintf("%s %s %s", (sccBus) ? "SCC2" : "", (activeNDA > 0) ? "NDA" : "", (radar_tracks) ? "RadarTracks" : "");

        //float accel = car_state.getAEgo();
#ifdef __TEST
        static int _ff = 0;
        if (_ff++ > 100) _ff = 0;
        if (_ff > 50) {
            limit_speed = 110;
            left_dist = _ff * 100;
        }
        else {
            roadLimitSpeed = 110;
        }
        cur_speed = 123;
#endif

        QColor color = QColor(255, 255, 255, 255);

        if (accel >= 0) {
            //int a = (int)(255.f - (180.f * (accel / 2.f)));
            //a = std::min(a, 255);
            //a = std::max(a, 80);
            //color = QColor(a, a, 255, 230);
        }
        else {
            int a = (int)(255.f - (255.f * (-accel / 3.f)));
            a = std::min(a, 255);
            a = std::max(a, 60);
            color = QColor(255, a, a, 255);
        }
        if (s->show_conn_info) {
            configFont(painter, "Inter", 35, "Bold");
            drawTextWithColor(painter, top_str.length() / 2 * 35 / 2 + 50, 40, top_str, color);
        }

        int bx = x;
        int by = y + 270;

        QString speed, str;
        speed.sprintf("%.0f", cur_speed);
        configFont(painter, "Inter", 110, "Bold");
        painter.setOpacity(1.0);
        drawTextWithColor(painter, bx, by+30, speed, color);

        painter.drawPixmap(bx - 100, by-60, 350, 150, ic_speed_bg);

        //color = QColor(255, 255, 255, 255);
#ifdef __TEST
        cruiseMaxSpeed = 110;
        enabled = true;
        longActiveUser = 2;
        applyMaxSpeed = 109;
        curveSpeed = 111;
#endif
        configFont(painter, "Inter", 60, "Bold");
        if (enabled && (longActiveUser > 0 || (longOverride && blinkerOn))) str.sprintf("%d", (int)(cruiseMaxSpeed + 0.5));
        else str = "--";
        color = QColor(0, 255, 0, 255);
        drawTextWithColor(painter, bx+170, by+15, str, color);
        if (enabled && longActiveUser>0 && applyMaxSpeed > 0 && applyMaxSpeed != cruiseMaxSpeed) {
            configFont(painter, "Inter", 50, "Bold");
            str.sprintf("%d", (int)(applyMaxSpeed + 0.5));
            drawTextWithColor(painter, bx + 250, by - 50, str, color);
        }
        if (enabled && curveSpeed > 0 && curveSpeed < 200) {
            configFont(painter, "Inter", 50, "Bold");
            str.sprintf("%d", (int)(curveSpeed + 0.5));
            color = QColor(255, 255, 0, 255);
            drawTextWithColor(painter, bx + 140, by + 110, str, color);
        }

#ifdef __TEST
        check_millis[5] = millis_since_boot();
#endif
        color = QColor(255, 255, 255, 255);
        QColor blackColor = QColor(0, 0, 0, 230);
        bx = x - 200;
        by = y + 250;
        if (limit_speed > 0) {
            QRect rectLimit(bx - 70, by - 70, 140, 140);
            painter.setBrush(QBrush(Qt::white));
            painter.drawEllipse(rectLimit);
            int padding = 10;
            rectLimit.adjust(padding, padding, -padding, -padding);
            painter.setBrush(Qt::NoBrush);
            painter.setPen(QPen(Qt::red, 12));
            painter.drawEllipse(rectLimit);
            configFont(painter, "Inter", 60, "Bold");
            str.sprintf("%d", limit_speed);
            drawTextWithColor(painter, bx, by + 20, str, blackColor);
            if (left_dist > 0) {
                configFont(painter, "Inter", 40, "Bold");
                if (left_dist < 1000) str.sprintf("%d m", left_dist);
                else  str.sprintf("%.1f km", left_dist / 1000.f);
                drawTextWithColor(painter, bx, by + 120, str, color);
            }
        }
        else if (roadLimitSpeed > 0 && roadLimitSpeed < 200) {
            painter.drawPixmap(bx - 60, by - 70, 120, 150, ic_road_speed);
            str.sprintf("%d", roadLimitSpeed);
            configFont(painter, "Inter", 50, "Bold");
            drawTextWithColor(painter, bx, by + 50, str, blackColor);
        }
    }
    // Tpms...
    if(s->show_tpms) {
      int bx = (btn_size - 24) / 2 + (bdr_s * 2);
      int by = rect().bottom() - footer_h / 2;
      auto tpms = car_state.getTpms();
      float fl = tpms.getFl();
      float fr = tpms.getFr();
      float rl = tpms.getRl();
      float rr = tpms.getRr();
#ifdef __TEST
      fl = 25;
      fr = 28;
      rl = 32;
      rr = 44;
      s->show_dm_info = 0;
#endif
      configFont(painter, "Inter", 38, "Bold");

      QFontMetrics fm(painter.font());
      QRect rcFont = fm.boundingRect("9");

      if (s->show_dm_info < 1) {
          int sx = 100;
          int sy = 220;
          bx = 110;
          by = height() - 100;
          painter.setPen(QPen(Qt::white, 3));
          painter.drawPixmap(bx - sx / 2, by - sy / 2, sx, sy, ic_tire);
          //painter.drawLine(bx, by - 40, bx, by + 40);
          //painter.drawLine(bx-50, by, bx+50, by);
          QColor tpmsColor = get_tpms_color(fl);
          drawTextWithColor(painter, bx-80, by-50, get_tpms_text(fl), tpmsColor);
          tpmsColor = get_tpms_color(fr);
          drawTextWithColor(painter, bx+80, by-50, get_tpms_text(fr), tpmsColor);
          tpmsColor = get_tpms_color(rl);
          drawTextWithColor(painter, bx-80, by+75, get_tpms_text(rl), tpmsColor);
          tpmsColor = get_tpms_color(rr);
          drawTextWithColor(painter, bx+80, by+75, get_tpms_text(rr), tpmsColor);
      }
      else {
          int center_x = bx - 30;
          int center_y = by - 0;
          int marginX = (int)(rcFont.width() * 3.2f);
          int marginY = (int)((footer_h / 2 - rcFont.height()) * 0.6f);
          drawText2(painter, center_x - marginX, center_y - marginY - rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
          drawText2(painter, center_x + marginX, center_y - marginY - rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
          drawText2(painter, center_x - marginX, center_y + marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
          drawText2(painter, center_x + marginX, center_y + marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
      }

    }
    if (s->show_radar_info) {
        QString str;
        //painter.setPen(Qt::NoPen);
        painter.setPen(QPen(Qt::white, 2));
        configFont(painter, "Inter", 40, "Bold");
        QFontMetrics fm(painter.font());
        QRect rcFont = fm.boundingRect("9");
        textColor = whiteColor(255);
        int w = rcFont.width();
        int wStr = 0;
        bool disp = false;
        for (auto const& vrd : s->scene.lead_vertices_ongoing) {
            auto [rx, ry, rd, rv, ry_rel] = vrd;
            disp = true;
            str.sprintf("%.0f", rv * 3.6);
            wStr = w * (str.length() + 1);
            QRect rectRadar(rx - wStr / 2, ry - 35, wStr, 42);
            bgColor = greenColor(255);
            painter.setBrush(bgColor);
            painter.drawRoundedRect(rectRadar, 15, 15);
            drawTextWithColor(painter, rx, ry, str, textColor);
            if (s->show_radar_info == 2) {
                str.sprintf("%.1f", ry_rel);
                drawTextWithColor(painter, rx, ry+40, str, textColor);
            }
        }
        for (auto const& vrd : s->scene.lead_vertices_oncoming) {
            auto [rx, ry, rd, rv, ry_rel] = vrd;
            str.sprintf("%.0f", rv * 3.6);
            wStr = w * (str.length() + 1);
            QRect rectRadar(rx - wStr / 2, ry - 35, wStr, 42);
            bgColor = redColor(255);
            painter.setBrush(bgColor);
            painter.drawRoundedRect(rectRadar, 15, 15);
            drawTextWithColor(painter, rx, ry, str, textColor);
            if (s->show_radar_info == 2) {
                str.sprintf("%.1f", ry_rel);
                drawTextWithColor(painter, rx, ry + 40, str, textColor);
            }
        }
        if (s->show_radar_info == 2) {
            for (auto const& vrd : s->scene.lead_vertices_stopped) {
                auto [rx, ry, rd, rv, ry_rel] = vrd;
                str = "*";
                wStr = w;
                if (true) {
                    QRect rectRadar(rx - wStr / 2, ry - 35, wStr, 42);
                    bgColor = blackColor(255);
                    painter.setBrush(bgColor);
                    painter.drawRoundedRect(rectRadar, 15, 15);
                    drawTextWithColor(painter, rx, ry, str, textColor);
                }
            }
        }
    }
    // 시간표시
    if(s->show_datetime) {
        QColor color = QColor(255, 255, 255, 230);
        if (s->show_datetime == 1 || s->show_datetime == 2) {
            configFont(painter, "Open Sans", 80, "Bold");
            //drawTextWithColor(painter, 150, height() - 400, QDateTime::currentDateTime().toString("hh:mm"), color);
            drawTextWithColor(painter, 150, 130, QDateTime::currentDateTime().toString("hh:mm"), color);
        }
        if (s->show_datetime == 1 || s->show_datetime == 3) {
            configFont(painter, "Open Sans", 45, "Bold");
            //drawTextWithColor(painter, 150, height() - 400 + 80, QDateTime::currentDateTime().toString("MM-dd-ddd"), color);
            drawTextWithColor(painter, 150, 130 + 80, QDateTime::currentDateTime().toString("MM-dd-ddd"), color);
        }
    }

}

void AnnotatedCameraWidget::drawHudApilot(QPainter& p, const cereal::ModelDataV2::Reader& model) {

    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setOpacity(1.);

    // Header gradient
    QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
    bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
    bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
    p.fillRect(0, 0, width(), header_h, bg);

    UIState* s = uiState();

    //const SubMaster& sm = *(s->sm);
    //const cereal::RadarState::Reader& radar_state = sm["radarState"].getRadarState();

    drawLaneLines(p, s);

    drawLeadApilot(p, model);

    //drawSteer(p);
    if(s->show_device_stat) drawDeviceState(p);
    //drawTurnSignals(p);
    //drawGpsStatus(p);
#ifdef __TEST
    drawDebugText(p);
#else
    if (s->show_debug) drawDebugText(p);
#endif

#if 0
    const auto controls_state = sm["controlsState"].getControlsState();
    //const auto car_params = sm["carParams"].getCarParams();
    const auto live_params = sm["liveParameters"].getLiveParameters();
    const auto car_control = sm["carControl"].getCarControl();
    const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
    const auto torque_state = controls_state.getLateralControlState().getTorqueState();

    QString infoText;
    infoText.sprintf("TP(%.2f/%.2f) LTP(%.2f/%.2f/%.0f) AO(%.2f/%.2f) SR(%.2f) SAD(%.2f) SCC(%d)",

        torque_state.getLatAccelFactor(),
        torque_state.getFriction(),

        live_torque_params.getLatAccelFactorRaw(),
        live_torque_params.getFrictionCoefficientRaw(),
        live_torque_params.getTotalBucketPoints(),

        live_params.getAngleOffsetDeg(),
        live_params.getAngleOffsetAverageDeg(),

        car_control.getSteerRatio(),
        car_control.getSteerActuatorDelay(),

        car_control.getSccBus()
    );

    // info

    p.save();
    configFont(p, "Inter", 34, "Regular");
    p.setPen(QColor(0xff, 0xff, 0xff, 200));
    p.drawText(rect().left() + 20, rect().height() - 15, infoText);
    p.restore();
#endif
    //drawBottomIcons(p);
}
