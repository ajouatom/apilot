#include "tools/cabana/canmessages.h"

#include <QDebug>
#include <QSettings>

#include "tools/cabana/dbcmanager.h"

CANMessages *can = nullptr;

CANMessages::CANMessages(QObject *parent) : QObject(parent) {
  can = this;

  QObject::connect(this, &CANMessages::received, this, &CANMessages::process, Qt::QueuedConnection);
  QObject::connect(&settings, &Settings::changed, this, &CANMessages::settingChanged);
}

CANMessages::~CANMessages() {
  replay->stop();
}

static bool event_filter(const Event *e, void *opaque) {
  CANMessages *c = (CANMessages *)opaque;
  return c->eventFilter(e);
}

bool CANMessages::loadRoute(const QString &route, const QString &data_dir, bool use_qcam) {
  routeName = route;
  replay = new Replay(route, {"can", "roadEncodeIdx", "carParams"}, {}, nullptr, use_qcam ? REPLAY_FLAG_QCAMERA : 0, data_dir, this);
  replay->setSegmentCacheLimit(settings.cached_segment_limit);
  replay->installEventFilter(event_filter, this);
  QObject::connect(replay, &Replay::segmentsMerged, this, &CANMessages::eventsMerged);
  if (replay->load()) {
    replay->start();
    return true;
  }
  return false;
}

QList<QPointF> CANMessages::findSignalValues(const QString &id, const Signal *signal, double value, FindFlags flag, int max_count) {
  auto evts = events();
  if (!evts) return {};

  auto l = id.split(':');
  int bus = l[0].toInt();
  uint32_t address = l[1].toUInt(nullptr, 16);

  QList<QPointF> ret;
  ret.reserve(max_count);
  for (auto &evt : *evts) {
    if (evt->which != cereal::Event::Which::CAN) continue;

    for (auto c : evt->event.getCan()) {
      if (bus == c.getSrc() && address == c.getAddress()) {
        double val = get_raw_value((uint8_t *)c.getDat().begin(), c.getDat().size(), *signal);
        if ((flag == EQ && val == value) || (flag == LT && val < value) || (flag == GT && val > value)) {
          ret.push_back({(evt->mono_time / (double)1e9) - can->routeStartTime(), val});
          if (ret.size() >= max_count)
            return ret;
        }
      }
    }
  }
  return ret;
}

void CANMessages::process(QHash<QString, CanData> *messages) {
  for (auto it = messages->begin(); it != messages->end(); ++it) {
    can_msgs[it.key()] = it.value();
  }
  delete messages;
  emit updated();
}

bool CANMessages::eventFilter(const Event *event) {
  static std::unique_ptr<QHash<QString, CanData>> new_msgs;
  static double prev_update_ts = 0;

  if (event->which == cereal::Event::Which::CAN) {
    if (!new_msgs) {
      new_msgs.reset(new QHash<QString, CanData>);
      new_msgs->reserve(1000);
    }

    double current_sec = replay->currentSeconds();
    if (counters_begin_sec == 0) {
      counters.clear();
      counters_begin_sec = current_sec;
    }

    auto can_events = event->event.getCan();
    for (const auto &c : can_events) {
      QString id = QString("%1:%2").arg(c.getSrc()).arg(c.getAddress(), 1, 16);

      std::lock_guard lk(lock);
      auto &list = received_msgs[id];
      while (list.size() > settings.can_msg_log_size) {
        list.pop_back();
      }
      CanData &data = list.emplace_front();
      data.ts = current_sec;
      data.bus_time = c.getBusTime();
      data.dat.append((char *)c.getDat().begin(), c.getDat().size());

      auto &count = counters[id];
      data.count = ++count;
      if (double delta = (current_sec - counters_begin_sec); delta > 0) {
        data.freq = count / delta;
      }
      (*new_msgs)[id] = data;
    }

    double ts = millis_since_boot();
    if ((ts - prev_update_ts) > (1000.0 / settings.fps)) {
      prev_update_ts = ts;
      // use pointer to avoid data copy in queued connection.
      emit received(new_msgs.release());
    }
  }
  return true;
}

const std::deque<CanData> CANMessages::messages(const QString &id) {
  std::lock_guard lk(lock);
  return received_msgs[id];
}

void CANMessages::seekTo(double ts) {
  replay->seekTo(ts, false);
  counters_begin_sec = 0;
}

void CANMessages::settingChanged() {
  replay->setSegmentCacheLimit(settings.cached_segment_limit);
}
