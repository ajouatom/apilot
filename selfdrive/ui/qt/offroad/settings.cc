#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>

#include "selfdrive/ui/qt/offroad/networking.h"

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "common/params.h"
#include "common/watchdog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QScroller>
#include <QListView>
#include <QListWidget>

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon, confirm
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("Experimental openpilot Longitudinal Control"),
      QString("<b>%1</b><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is experimental for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when using experimental openpilot longitudinal control.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "SccConnectedBus2",
      "SCC Module connected BUS2",
      "",
      "../assets/offroad/icon_warning.png",
    },
    {
      "EnableRadarTracks",
      "EnableRadarTracks",
      "Using RadarTracks instead of SCC data. SANTAFE2022HEV",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h Format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
    {
      "NavSettingLeftSide",
      tr("Show Map on Left Side of UI"),
      tr("Show map on left side when in split screen view."),
      "../assets/offroad/icon_road.png",
    },
#endif
  };

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = false;// params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("🌮 End-to-End Longitudinal Control 🌮"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                       "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; mistakes should be expected."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner."));

  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable()) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable());

    const bool op_long = CP.getOpenpilotLongitudinalControl() && !CP.getExperimentalLongitudinalAvailable();
    const bool exp_long_enabled = CP.getExperimentalLongitudinalAvailable() && params.getBool("ExperimentalLongitudinalEnabled");
    if (op_long || exp_long_enabled) {
      // normal description and toggle
      e2e_toggle->setEnabled(true);
      e2e_toggle->setDescription(e2e_description);
    } else {
      // no long for now
      e2e_toggle->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString no_long = tr("Experimental mode is currently unavailable on this car, since the car's stock ACC is used for longitudinal control.");
      const QString exp_long = tr("Enable experimental longitudinal control to allow experimental mode.");
      e2e_toggle->setDescription("<b>" + (CP.getExperimentalLongitudinalAvailable() ? exp_long : no_long) + "</b><br><br>" + e2e_description);
    }

    e2e_toggle->refresh();
  } else {
    e2e_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(true);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton(tr("Soft restart"));
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton(tr("Reset Calibration"));
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration and live params?"), tr("Reset"), this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  addItem(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      Params().put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
        //btn->setEnabled(offroad);
        btn->setEnabled(true);
    }
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *rebuild_btn = new QPushButton(tr("Rebuild"));
  rebuild_btn->setObjectName("rebuild_btn");
  power_layout->addWidget(rebuild_btn);
  QObject::connect(rebuild_btn, &QPushButton::clicked, [=]() {

    if (ConfirmationDialog::confirm(tr("Are you sure you want to rebuild?"), tr("Reset"), this)) {
      std::system("cd /data/openpilot && scons -c");
      std::system("rm /data/openpilot/.sconsign.dblite");
      std::system("rm /data/openpilot/prebuilt");
      std::system("rm -rf /tmp/scons_cache");
      if (Hardware::TICI())
        std::system("sudo reboot");
      else
        std::system("reboot");
    }
  });

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (!Hardware::PC()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #rebuild_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #rebuild_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

static QStringList get_list(const char* path)
{
  QStringList stringList;
  QFile textFile(path);
  if(textFile.open(QIODevice::ReadOnly))
  {
      QTextStream textStream(&textFile);
      while (true)
      {
        QString line = textStream.readLine();
        if (line.isNull())
            break;
        else
            stringList.append(line);
      }
  }

  return stringList;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton(tr("← Back"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      font-weight: bold;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(300, 110);
  sidebar_layout->addSpacing(10);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignRight);
  sidebar_layout->addSpacing(10);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);
  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), new Networking(this)},
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {"크루즈", new CruisePanel(this)},
    {"튜닝", new TuningPanel(this)},
    {"기타", new CommunityPanel(this)},
  };

#ifdef ENABLE_MAPS
  //auto map_panel = new MapPanel(this);
  //panels.push_back({tr("Navigation"), map_panel});
  //QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 60px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}



/////////////////////////////////////////////////////////////////////////

CommunityPanel::CommunityPanel(QWidget* parent) : QWidget(parent) {

  main_layout = new QStackedLayout(this);

  homeScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
  vlayout->setContentsMargins(0, 20, 0, 20);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  QPushButton* selectCarBtn = new QPushButton(selected.length() ? selected : tr("Select your car"));
  selectCarBtn->setObjectName("selectCarBtn");
  //selectCarBtn->setStyleSheet("margin-right: 30px;");
  //selectCarBtn->setFixedSize(350, 100);
  connect(selectCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(selectCar); });

  homeWidget = new QWidget(this);
  QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
  homeWidget->setObjectName("homeWidget");

  ScrollView *scroller = new ScrollView(homeWidget, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  main_layout->addWidget(homeScreen);

  selectCar = new SelectCar(this);
  connect(selectCar, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(selectCar, &SelectCar::selectedCar, [=]() {

     QString selected = QString::fromStdString(Params().get("SelectedCar"));
     selectCarBtn->setText(selected.length() ? selected : tr("Select your car"));
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(selectCar);
  QHBoxLayout* layoutBtn = new QHBoxLayout(homeWidget);

  layoutBtn->addWidget(selectCarBtn);
  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn, 0);
  
  auto tmuxlog_btn = new ButtonControl("Tmux error log", tr("RUN"));
  QObject::connect(tmuxlog_btn, &ButtonControl::clicked, [=]() {
    const std::string txt = util::read_file("/data/tmux_error.log");
    ConfirmationDialog::alert(QString::fromStdString(txt), this);
  });
  vlayout->addWidget(tmuxlog_btn);

  vlayout->addWidget(scroller, 1);

  auto updateBtn = new ButtonControl("업데이트 체크 및 적용", "업데이트");
  QObject::connect(updateBtn, &ButtonControl::clicked, [=]()
  {
      const char* gitcommit = "/data/openpilot/selfdrive/assets/addon/sh/gitcommit.sh";
      const char* gitpull = "/data/openpilot/selfdrive/assets/addon/sh/gitpull.sh";


      std::system(gitcommit);
      std::system("date '+%F %T' > /data/params/d/LastUpdateTime");
      QString desc = "";
      QString commit_local = QString::fromStdString(Params().get("GitCommit").substr(0, 7));
      QString commit_remote = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 7));

      desc += QString("(로컬/리모트): %1/%2\n").arg(commit_local, commit_remote);
      if (commit_local == commit_remote) {
          desc += QString("로컬과 리모트가 일치합니다.");
      }
      else {
          desc += QString("업데이트가 있습니다.");
      }
      if (ConfirmationDialog::alert(desc, this)) {
          //Params().putBool("OpkrPrebuiltOn", 0);
          std::system("cd /data/openpilot; rm -f prebuilt");
          std::system(gitpull);
      }
  });
  toggleLayout->addWidget(updateBtn);
  toggleLayout->addWidget(new CPrebuiltToggle());

  // 기타 (Community)
  //toggleLayout->addWidget(horizontal_line());
  //toggleLayout->addWidget(new ParamControl("SccConnectedBus2", "SCC배선이 BUS2에 연결됨", "SCC배선을 개조하여 BUS2에 연결된경우 켭니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamControl("EnableAutoEngage", "EnableAutoEngage", "", "../assets/offroad/icon_road.png", this));

  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new CValueControl("AutoNaviSpeedCtrl", "과속카메라작동 방법(2)", "0:사용안함, 1:NDA, 2:NDA + 순정네비게이션", "../assets/offroad/icon_road.png", 0, 2, 1));
  //toggleLayout->addWidget(new CValueControl("AutoRoadLimitCtrl", "NDA: 속도제한(0:None,1:Limit,2:Apply)", "Limit: 속도를 제한합니다. Apply: 제한속도로 실시간 적용합니다.", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("NaviSpeedLimitDecelRate", "차량네비속도제한 감속도(50%)", "거리정보가 없어 감속율로 임시적용합니다.", "../assets/offroad/icon_road.png", 0, 100, 5));
  toggleLayout->addWidget(new CValueControl("LongControlActiveSound", "크루즈 소리 0:OFF,1:Half, 2:ON", "크루즈 소리를 켭니다.", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new ParamControl("CustomMapbox", "CustomMapBox입력", "http://IP주소:8082 에 접속하여 mapbox token을 입력하면 자동으로 켜집니다. 끄면, 초기화됩니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamControl("ShowDebugUI", "Show Debug UI", "", "../assets/offroad/icon_shell.png", this));
  toggleLayout->addWidget(new ParamControl("ShowDateTime", "시간정보표시", "", "../assets/offroad/icon_shell.png", this));
  toggleLayout->addWidget(new ParamControl("KeepEngage", "인게이지 유지모드", "", "../assets/offroad/icon_shell.png", this));
  toggleLayout->addWidget(new ParamControl("UseLanelines", "레인모드사용", "", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("PathOffset", "차선치우침 좌우보정", "(-)좌측, (+)우측", "../assets/offroad/icon_road.png", -200, 200, 1));
  toggleLayout->addWidget(new CValueControl("HapticFeedbackWhenSpeedCamera", "핸들햅틱기능사용", "0:사용안함,1:진동,2:계기판,3:HUD표시", "../assets/offroad/icon_road.png", 0, 3, 1));
  toggleLayout->addWidget(new CValueControl("SoftHoldMode", "소프트오토홀드기능(1)", "0:사용안함,1:사용,2:SCC제어와함께사용(단,사이드가 걸리는 차량이 있음)", "../assets/offroad/icon_road.png", 0, 2, 1));
}

TuningPanel::TuningPanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);

    homeScreen = new QWidget(this);
    QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
    vlayout->setContentsMargins(0, 20, 0, 20);

    homeWidget = new QWidget(this);
    QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
    homeWidget->setObjectName("homeWidget");

    ScrollView* scroller = new ScrollView(homeWidget, this);
    scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    main_layout->addWidget(homeScreen);

    vlayout->addWidget(scroller, 1);

    // 튜닝
    toggleLayout->addWidget(new CValueControl("StopDistance", "StopDistance(600cm)", "선행차와 정지하는 거리를 입력합니다.", "../assets/offroad/icon_road.png", 200, 1000, 50));
    toggleLayout->addWidget(new CValueControl("TrafficStopDistanceAdjust", "신호정지 위치 조정(200cm)", "+값으로 하면 정지선에 다가갑니다.", "../assets/offroad/icon_road.png", -1000, 1000, 10));
    toggleLayout->addWidget(horizontal_line());    
    toggleLayout->addWidget(new CValueControl("LiveSteerRatioApply", "종컨: LiveSteerRatioApply(100)", "", "../assets/offroad/icon_road.png", 50, 110, 1));
    toggleLayout->addWidget(new CValueControl("SteeringRateCost", "종컨: SteeringRateCost(800)", "", "../assets/offroad/icon_road.png", 10, 2000, 10));
    toggleLayout->addWidget(new CValueControl("SteerDeltaUp", "종컨: SteerDeltaUp(3)", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    toggleLayout->addWidget(new CValueControl("SteerDeltaDown", "종컨: SteerDeltaDown(7)", "", "../assets/offroad/icon_road.png", 1, 20, 1));
    toggleLayout->addWidget(new CValueControl("PathCostApply", "종건: PathCostApply(100)", "", "../assets/offroad/icon_road.png", 90, 200, 5));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("JerkUpperLowerLimit", "롱컨: JERK값(8)", "값이 커지면 가감속반응이 빨라지지만, 기분이 안좋음.", "../assets/offroad/icon_road.png", 5, 50, 1));
    toggleLayout->addWidget(new CValueControl("LongitudinalTuningKf", "롱컨: FF게인(110%)", "(시험용) ACCEL을 좀 더 강력하게 적용합니다.", "../assets/offroad/icon_road.png", 100, 120, 1));
    toggleLayout->addWidget(new CValueControl("LongitudinalTuningKpV", "롱컨: P게인(100)", "(시험용) ", "../assets/offroad/icon_road.png", 50, 150, 1));
    toggleLayout->addWidget(new CValueControl("LongitudinalTuningKiV", "롱컨: I게인(0)", "(시험용) ", "../assets/offroad/icon_road.png", 0, 200, 5));
    toggleLayout->addWidget(new CValueControl("StartAccelApply", "가속초기속도2.0x(0%)", "정지->출발시 가속도의 가속율을 지정합니다 0: 사용안함.", "../assets/offroad/icon_road.png", 0, 100, 10));
    toggleLayout->addWidget(new CValueControl("StopAccelApply", "정지유지브레이크-2.0x(0%)", "정지유지시 브레이크압을 조정합니다. 0: 사용안함. ", "../assets/offroad/icon_road.png", 0, 100, 10));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("XEgoObstacleCost", "X_EGO_COST(5)", "증가할수록 정지선정지가 정확해지나, 급감속이 강해집니다.", "../assets/offroad/icon_road.png", 3, 50, 1));
    toggleLayout->addWidget(new CValueControl("JEgoCost", "J_EGO_COST(5)", "", "../assets/offroad/icon_road.png", 4, 10, 1));
    toggleLayout->addWidget(new CValueControl("AChangeCost", "A_CHANGE_COST(150)", "적으면 선행차에 대한 반응이 강해집니다. ", "../assets/offroad/icon_road.png", 20, 400, 10));
    toggleLayout->addWidget(new CValueControl("DangerZoneCost", "DANGER_ZONE_COST(100)", "", "../assets/offroad/icon_road.png", 0, 400, 10));
    toggleLayout->addWidget(new CValueControl("LeadDangerFactor", "LEAD_DANGER_FACTOR(75)", "", "../assets/offroad/icon_road.png", 75, 100, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("ApplyLongDynamicCost", "차량간격유지 동적제어(OFF)", "전방차량의 간격을 최대한 유지하도록 응답속도가 빨라집니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollow", "차량간격제어:상대속도-(110%)", "선행차와 점점 가까와지면 차량거리를 안전하게 증가시키도록 합니다.", "../assets/offroad/icon_road.png", 100, 300, 5));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollowApart", "차량간격제어:상대속도+(95%)", "선행차와 점점 멀어지면 차량거리를 줄이도록 합니다.", "../assets/offroad/icon_road.png", 20, 100, 5));
    toggleLayout->addWidget(new CValueControl("ApplyDynamicTFollowDecel", "차량간격제어:감속(110%)", "차량이 급감속 할 수록 차량간격을 벌리도록 제어합니다.", "../assets/offroad/icon_road.png", 100, 300, 5));
    toggleLayout->addWidget(new CValueControl("TFollowRatio", "위험:차량간격비율(100%)", "선행차와의 간격을 조정합니다. 100%이하로 하면 매우 위험합니다.", "../assets/offroad/icon_road.png", 70, 120, 5));
    
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("TrafficStopAccel", "신호정지 감속율 (80%)", "신호를 만나면 서서히 감속하여 정지합니다.", "../assets/offroad/icon_road.png", 10, 120, 10));
    //toggleLayout->addWidget(new ParamControl("TrafficStopModelSpeed", "신호정지 모델속도(OFF)", "신호정지시 모델에서 제공하는 속도를 따름니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("E2eDecelSpeed", "모델의 자동속도 적용(90)", "지정속도 이하에서는 모델이 제공하는 속도를 적용합니다. 0: 적용안함.", "../assets/offroad/icon_road.png", 0, 120, 10));
    toggleLayout->addWidget(horizontal_line());
    //toggleLayout->addWidget(new CValueControl("AccelLimitEcoSpeed", "초기가속제한 속도(3km/h)", "지정속도까지 가속도를 제한합니다. 출발시 충격 방지!!", "../assets/offroad/icon_road.png", 0, 100, 1));
    //toggleLayout->addWidget(new ParamControl("AccelLimitConfusedModel", "모델혼잡시 조향가속비율적용(ON)", "E2E모드에서 모델예측이 20M이내인경우 가속을 제한합니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AccelBoost", "가속도 제어(100%)", "가속도를 제어합니다.", "../assets/offroad/icon_road.png", 50, 200, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new CValueControl("AutoCurveSpeedCtrl", "모델커브속도조절(1)", "곡선도로를 만나면 속도를 줄여줍니다. 0:사용안함,1:방법1,2:방법2", "../assets/offroad/icon_road.png", 0, 2, 1));
    toggleLayout->addWidget(new CValueControl("AutoCurveSpeedFactor", "모델커브속도조절비율(100%)", "커브속도조절(커브속도 조절 2일때 적용)", "../assets/offroad/icon_road.png", 50, 150, 1));

}
CruisePanel::CruisePanel(QWidget* parent) : QWidget(parent) {

    main_layout = new QStackedLayout(this);

    homeScreen = new QWidget(this);
    QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
    vlayout->setContentsMargins(0, 20, 0, 20);

    homeWidget = new QWidget(this);
    QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
    homeWidget->setObjectName("homeWidget");

    ScrollView* scroller = new ScrollView(homeWidget, this);
    scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    main_layout->addWidget(homeScreen);

    vlayout->addWidget(scroller, 1);

    // 크루즈
    toggleLayout->addWidget(new ParamControl("AutoSyncCruiseSpeed", "가속시 크루즈속도를 맞춤", "가속시 주행속도가 크루즈 속도보다 높아지면 맞춰줍니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoSpeedUptoRoadSpeedLimit", "자동속도증가모드 (100%)", "전방차량의 속도가 빨라지면 RoadSpeedLimit까지 속도를 올립니다.", "../assets/offroad/icon_road.png", 0, 200, 10));
    toggleLayout->addWidget(new CValueControl("AutoSpeedAdjustWithLeadCar", "선행차에 크루즈속도맞춤(+40)", "불필요한 기능, 선행차량의 속도에 옵셋속도를 더한 속도를 설정합니다.", "../assets/offroad/icon_road.png", 0, 100, 5));
    toggleLayout->addWidget(new CValueControl("InitMyDrivingMode", "드라이브모드:초기값(3)", "1:연비모드,2:안전모드,3:일반주행모드,4:고속모드(신호무시)", "../assets/offroad/icon_road.png", 1, 4, 1));
    toggleLayout->addWidget(new CValueControl("MyEcoModeFactor", "드라이브모드:연비가속비율(80%)", "일반모드대비 가속비율을 지정합니다.", "../assets/offroad/icon_road.png", 50, 95, 5));
    toggleLayout->addWidget(new CValueControl("MySafeModeFactor", "드라이브모드:안전비율(80%)", "가속도, 정지거리, 감속율, 차간거리를 평소보다 안전하게 유지합니다.", "../assets/offroad/icon_road.png", 50, 90, 10));
    toggleLayout->addWidget(new CValueControl("CruiseButtonMode", "크루즈버튼작동모드", "0:일반속도제어,1:사용자속도제어1, 2:사용자속도제어2, 3:사용자속도제어3, 4:사용자속도제어4", "../assets/offroad/icon_road.png", 0, 4, 1));
    toggleLayout->addWidget(new CValueControl("GapButtonMode", "크루즈갭버튼작동모드", "0:1,2,3,4(오토),1:1,2,3,4(오토),5(크루즈OFF),2:4(오토),5(크루즈OFF)", "../assets/offroad/icon_road.png", 0, 2, 1));
    toggleLayout->addWidget(new CValueControl("PrevCruiseGap", "크루즈갭 :초기값(4)", "4: 자동", "../assets/offroad/icon_road.png", 1, 4, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("AutoResumeFromGas", "엑셀 크루즈ON 사용", "엑셀밟으면 크루즈를 켭니다. 60%이상 밟으면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeed", "엑셀 크루즈ON:속도", "설정속도이상이 되면 자동으로 크루즈를 켭니다.", "../assets/offroad/icon_road.png", 20, 140, 5));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeedMode", "엑셀 크루즈ON:속도복원설정(0)", "0:현재속도, 1:이전속도, 2: 선행차량이 있을때만 이전속도", "../assets/offroad/icon_road.png", 0, 2, 1));
    toggleLayout->addWidget(new CValueControl("AutoCancelFromGas", "엑셀 크루즈OFF:속도", "설정속도이하에서 엑셀을 밟으면 크루즈가 해제됩니다.", "../assets/offroad/icon_road.png", 0, 140, 1));
    toggleLayout->addWidget(horizontal_line());
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeRelease", "브레이크해제 크루즈ON 사용", "브레이크를 떼면 크르즈를 켭니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeReleaseDist", "브레이크해제 크루즈ON:주행중,선행차", "0:사용안함. 브레이크를 떼고, 선행차가 일정 거리이상이면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", 0, 80, 5));
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseLeadCar", "브레이크해제 크루즈ON:정지상태,선행차", "정지상태에서 선행차량이 10M이내이면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", this));
    toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeCarSpeed", "브레이크해제 크루즈ON:주행중,속도", "0:사용안함. 브레이크를 떼고, 일정속도이상이면 크루즈를 현재 속도로 켭니다.", "../assets/offroad/icon_road.png", 0, 80, 5));
    toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseTrafficSign", "브레이크해제 크루즈ON:주행중,신호", "60Km/h이하에서 브레이크를 떼고 신호가 감지되면 현재속도로 크루즈를 켭니다.", "../assets/offroad/icon_road.png", this));

}

// ajouatom
CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit/*=1*/) : AbstractControl(title, desc, icon)
{

    m_params = params;
    m_min = min;
    m_max = max;
    m_unit = unit;

    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    hlayout->addWidget(&label);

    btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnminus.setFixedSize(150, 100);
    btnplus.setFixedSize(150, 100);
    hlayout->addWidget(&btnminus);
    hlayout->addWidget(&btnplus);

    QObject::connect(&btnminus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value - m_unit;
        if (value < m_min) {
            value = m_min;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });

    QObject::connect(&btnplus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value + m_unit;
        if (value > m_max) {
            value = m_max;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });
    refresh();
}

void CValueControl::refresh()
{
    label.setText(QString::fromStdString(Params().get(m_params.toStdString())));
    btnminus.setText("－");
    btnplus.setText("＋");
}

GitHash::GitHash() : AbstractControl("커밋(로컬/리모트)", "", "") {

    QString lhash = QString::fromStdString(Params().get("GitCommit").substr(0, 10));
    QString rhash = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10));
    hlayout->addStretch(2);

    local_hash.setText(QString::fromStdString(Params().get("GitCommit").substr(0, 10)));
    remote_hash.setText(QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10)));
    local_hash.setAlignment(Qt::AlignVCenter);
    remote_hash.setAlignment(Qt::AlignVCenter);
    local_hash.setStyleSheet("color: #aaaaaa");
    if (lhash == rhash) {
        remote_hash.setStyleSheet("color: #aaaaaa");
    }
    else {
        remote_hash.setStyleSheet("color: #0099ff");
    }
    hlayout->addWidget(&local_hash);
    hlayout->addWidget(&remote_hash);
}
SelectCar::SelectCar(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button
  QPushButton* back = new QPushButton(tr("Back"));
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  //list->setAttribute(Qt::WA_AcceptTouchEvents, true);
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  list->addItem(tr("[ Not selected ]"));

  QStringList items = get_list("/data/params/d/SupportedCars");
  list->addItems(items);
  list->setCurrentRow(0);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  int index = 0;
  for(QString item : items) {
    if(selected == item) {
        list->setCurrentRow(index + 1);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){

    if(list->currentRow() == 0)
        Params().remove("SelectedCar");
    else
        Params().put("SelectedCar", list->currentItem()->text().toStdString());

    emit selectedCar();
    });

  main_layout->addWidget(list);
}
