#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"

#include <QTimer>
#include <QMap>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"


// ***** onroad widgets *****

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  QPixmap engage_img;
  QPixmap experimental_img;
  bool v_ego_cluster_seen = false;
  int skip_frame_count = 0;
  bool wide_cam_requested = false;
protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter& painter, const UIState* s);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
  FirstOrderFilter accel_filter;

  // neokii
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float rotate=0.0);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color);
  void drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color);
  void paintEvent(QPaintEvent *event) override;

  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;


  uint64_t last_update_params;

  // neokii
  QPixmap ic_brake;
  QPixmap ic_autohold_warning;
  QPixmap ic_autohold_active;
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;
  QPixmap ic_turn_signal_l;
  QPixmap ic_turn_signal_r;
  QPixmap ic_satellite;
  QPixmap ic_trafficLight_green;
  QPixmap ic_trafficLight_red;

  QMap<QString, QPixmap> ic_oil_com;

  void drawMaxSpeed(QPainter &p);
  void drawSpeed(QPainter &p);
  void drawBottomIcons(QPainter &p);
  void drawSteer(QPainter &p);
  void drawDeviceState(QPainter &p);
  void drawTurnSignals(QPainter &p);
  void drawGpsStatus(QPainter &p);
  void drawDebugText(QPainter &p);
  void drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model);

  int m_fps = 0;
  float m_cur_speed = 0;
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

  void paintEvent(QPaintEvent *event) override;

private:
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

  // neokii
private:
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
