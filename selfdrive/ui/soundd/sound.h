#pragma once

#include <tuple>

#include <QMap>
#include <QSoundEffect>
#include <QString>

#include "system/hardware/hw.h"
#include "selfdrive/ui/ui.h"


const float MAX_VOLUME = 1.0;

const std::tuple<AudibleAlert, QString, int, float> sound_list[] = {
  // AudibleAlert, file name, loop count
  {AudibleAlert::ENGAGE, "engage.wav", 0, MAX_VOLUME},
  {AudibleAlert::DISENGAGE, "disengage.wav", 0, MAX_VOLUME},
  {AudibleAlert::REFUSE, "refuse.wav", 0, MAX_VOLUME},

  {AudibleAlert::PROMPT, "prompt.wav", 0, MAX_VOLUME},
  {AudibleAlert::PROMPT_REPEAT, "prompt.wav", QSoundEffect::Infinite, MAX_VOLUME},
  {AudibleAlert::PROMPT_DISTRACTED, "prompt_distracted.wav", QSoundEffect::Infinite, MAX_VOLUME},

  {AudibleAlert::WARNING_SOFT, "warning_soft.wav", QSoundEffect::Infinite, MAX_VOLUME},
  //{AudibleAlert::WARNING_IMMEDIATE, "warning_immediate.wav", QSoundEffect::Infinite, MAX_VOLUME},
  {AudibleAlert::WARNING_IMMEDIATE, "warning_immediate.wav", 2, MAX_VOLUME},
  {AudibleAlert::LONG_ENGAGED, "tici_engaged.wav", 0, MAX_VOLUME},
  {AudibleAlert::LONG_DISENGAGED, "tici_disengaged.wav", 0, MAX_VOLUME},
  {AudibleAlert::TRAFFIC_SIGN_GREEN, "traffic_sign_green.wav", 0, MAX_VOLUME},
  {AudibleAlert::TRAFFIC_SIGN_CHANGED, "traffic_sign_changed.wav", 0, MAX_VOLUME},
  {AudibleAlert::TRAFFIC_ERROR, "audio_traffic_error.wav", 0, MAX_VOLUME},
  {AudibleAlert::BSD_WARNING, "audio_car_watchout.wav", 0, MAX_VOLUME},
  {AudibleAlert::LANE_CHANGE, "audio_lane_change.wav", 0, MAX_VOLUME},
  {AudibleAlert::STOP_STOP, "audio_stopstop.wav", 0, MAX_VOLUME},
  {AudibleAlert::STOPPING, "audio_stopping.wav", 0, MAX_VOLUME},
  {AudibleAlert::AUTO_HOLD, "audio_auto_hold.wav", 0, MAX_VOLUME},
  {AudibleAlert::ENGAGE2, "audio_engage.wav", 0, MAX_VOLUME},
  {AudibleAlert::DISENGAGE2, "audio_disengage.wav", 0, MAX_VOLUME},
  {AudibleAlert::SPEED_DOWN, "audio_speed_down.wav", 0, MAX_VOLUME},
  {AudibleAlert::AUDIO_TURN, "audio_turn.wav", 0, MAX_VOLUME},
};

class Sound : public QObject {
public:
  explicit Sound(QObject *parent = 0);

protected:
  void update();
  void setAlert(const Alert &alert);

  SubMaster sm;
  Alert current_alert = {};
  QMap<AudibleAlert, QPair<QSoundEffect *, int>> sounds;
  int current_volume = -1;
};
