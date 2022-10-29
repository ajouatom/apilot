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
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
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
      tr("Disengage On Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "EndToEndLong",
      tr("🌮 End-to-end longitudinal (extremely alpha) 🌮"),
      "",
      "../assets/offroad/icon_road.png",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("Experimental openpilot longitudinal control"),
      tr("<b>WARNING: openpilot longitudinal control is experimental for this car and will disable AEB.</b>"),
      "../assets/offroad/icon_speed_limit.png",
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

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["EndToEndLong"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would. Super experimental.");

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
      params.remove("EndToEndLong");

      const QString no_long = tr("openpilot longitudinal control is not currently available for this car.");
      const QString exp_long = tr("Enable experimental longitudinal control to enable this.");
      e2e_toggle->setDescription("<b>" + (CP.getExperimentalLongitudinalAvailable() ? exp_long : no_long) + "</b><br><br>" + e2e_description);
    }

    e2e_toggle->refresh();
  } else {
    e2e_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(false);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), this)) {
      params.remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      RichTextDialog::alert(QString::fromStdString(txt), this);
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
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), this)) {
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
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), this)) {
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
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
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
  QPushButton *close_btn = new QPushButton(tr("×"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), new Networking(this)},
    {tr("Toggles"), new TogglesPanel(this)},
    {tr("Software"), new SoftwarePanel(this)},
    {tr("Community"), new CommunityPanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({tr("Navigation"), map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
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
    RichTextDialog::alert(QString::fromStdString(txt), this);
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
      if (ConfirmationDialog::confirm(desc, this)) {
          //Params().putBool("OpkrPrebuiltOn", 0);
          std::system("cd /data/openpilot; rm -f prebuilt");
          std::system(gitpull);
      }
  });
  toggleLayout->addWidget(updateBtn);
  toggleLayout->addWidget(new CPrebuiltToggle());

  toggleLayout->addWidget(horizontal_line());
  
  toggleLayout->addWidget(new CValueControl("StopDistance", "StopDistance(600cm)", "선행차와 정지하는 거리를 입력합니다.", "../assets/offroad/icon_road.png", 200, 1000, 50));
  toggleLayout->addWidget(new CValueControl("LongitudinalActuatorDelayUpperBound", "ActuatorDelayUpperBound(0.5*100%)", "", "../assets/offroad/icon_road.png", 10, 200, 1));
  toggleLayout->addWidget(new CValueControl("LongitudinalActuatorDelayLowerBound", "ActuatorDelayLowerBound(0.5*100%)", "", "../assets/offroad/icon_road.png", 10, 200, 1));
  toggleLayout->addWidget(new CValueControl("XEgoObstacleCost", "X_EGO_OBSTACLE_COST(3)", "증가할수록 정지선정지가 정확해지나, 급감속이 강해집니다.", "../assets/offroad/icon_road.png", 0, 100, 1));
  toggleLayout->addWidget(new CValueControl("JEgoCost", "J_EGO_COST(5)", "", "../assets/offroad/icon_road.png", 0, 100, 1));
  toggleLayout->addWidget(new CValueControl("AChangeCost", "A_CHANGE_COST(150)", "적으면 선행차에 대한 반응이 강해집니다. 차량간격유지 동적제어를 켜면 130정도로 하십시오", "../assets/offroad/icon_road.png", 0, 400, 10));
  toggleLayout->addWidget(new CValueControl("DangerZoneCost", "DANGER_ZONE_COST(100)", "", "../assets/offroad/icon_road.png", 0, 400, 10));
  toggleLayout->addWidget(horizontal_line());

#if 1
  
  toggleLayout->addWidget(new ParamControl("AutoSyncCruiseSpeed", "가속시 크루즈속도를 맞춤", "가속시 주행속도가 크루즈 속도보다 높아지면 맞춰줍니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("InitCruiseGap", "크루즈갭 초기값(3)", "1:연비모드,2:관성제어모드,3:일반주행모드,4:E2E OFF모드", "../assets/offroad/icon_road.png", 1, 4, 1));
  toggleLayout->addWidget(new CValueControl("CruiseButtonMode", "크루즈버튼작동모드", "0:일반속도제어,1:관성주행모드(-)버튼이용.", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("TrafficStopAccel", "신호정지 감속율 (70%)", "신호를 만나면 서서히 감속하여 정지합니다.", "../assets/offroad/icon_road.png", 10, 120, 10));
  toggleLayout->addWidget(new CValueControl("AutoSpeedAdjustWithLeadCar", "선행차속도에 크루즈속도맞추기(+40km/h)", "선행차량의 속도에 옵셋속도를 더한 속도를 설정합니다.", "../assets/offroad/icon_road.png", 0, 100, 5));
  toggleLayout->addWidget(new CValueControl("AccelLimitEconomy", "연비운전 가속비율(60%)", "연비운전시(크루즈갭1,2) 가속비율을 설정합니다.", "../assets/offroad/icon_road.png", 10, 100, 10));
  toggleLayout->addWidget(new CValueControl("AccelLimitTurn", "조향가속비율(100%)", "조향시 가속율을 줄여 급가속을 피해줍니다.", "../assets/offroad/icon_road.png", 10, 100, 10));
  toggleLayout->addWidget(new ParamControl("AccelLimitConfusedModel", "모델혼잡시 조향가속비율적용", "E2E모드에서 모델예측이 20M이내인경우 가속을 제한합니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AccelBoost", "가속도 제어(100%)", "가속도를 제어합니다. 크루즈갭:3일 때 만 적용됨 ", "../assets/offroad/icon_road.png", 50, 200, 10));
  toggleLayout->addWidget(new CValueControl("TrafficStopDistanceAdjust", "신호정지 위치 조정(450cm)", "+값으로 하면 정지선에 다가갑니다.", "../assets/offroad/icon_road.png", -1000, 1000, 10));
  toggleLayout->addWidget(new CValueControl("AutoSpeedUptoRoadSpeedLimit", "자동속도증가모드 (100%)", "전방차량의 속도가 빨라지면 RoadSpeedLimit까지 속도를 올립니다.", "../assets/offroad/icon_road.png", 0, 200, 10));
  toggleLayout->addWidget(new ParamControl("ApplyLongDynamicCost", "차량간격유지 동적제어", "전방차량의 간격을 최대한 유지하도록 응답속도가 빨라집니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeed", "속도크루즈ON:속도", "설정속도이상이 되면 자동으로 크루즈를 켭니다.", "../assets/offroad/icon_road.png", 20, 40, 5));
  toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeRelease", "브레이크해제 크루즈ON 사용", "브레이크를 떼면 크르즈를 켭니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeReleaseDist", "브레이크해제 크루즈ON:선행차거리", "브레이크를 떼고, 선행차가 거리이상이면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", 0, 80, 5));
  toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseLeadCar", "브레이크해제 크루즈ON:정지상태,선행차10M이내", "정지상태에서 선행차량이 10M이내이면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("AutoResumeFromGas", "엑셀크루즈ON", "엑셀을 60%이상 밟으면 크루즈를 켭니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamControl("AutoCurveSpeedCtrl", "모델커브속도조절", "곡선도로를 만나면 속도를 줄여줍니다.", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoCurveSpeedFactor", "모델커브속도조절비율(95%)", "적으면 속도를 많이 줄입니다.", "../assets/offroad/icon_road.png", 50, 150, 1));
  toggleLayout->addWidget(new ParamControl("AutoNaviSpeedCtrl", "NDA 지원", "별도의 단말기에 NDA Manager를 설치하고 같은 네트웍에 물리고 Tmap을 실행하세요", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoRoadLimitCtrl", "NDA: 속도제한(0:None,1:Limit,2:Apply)", "Limit: 속도를 제한합니다. Apply: 제한속도로 실시간 적용합니다.", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("LongControlActiveSound", "크루즈 소리 0:OFF,1:Half, 2:ON", "크루즈 소리를 켭니다.", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new ParamControl("ShowDebugUI", "Show Debug UI", "", "../assets/offroad/icon_shell.png", this));

#else
  toggleLayout->addWidget(new CValueControl("AutoResumeFromGasSpeed", "CruiseON:Gas_Speed", "Enable Cruise control from Gas, Speed", "../assets/offroad/icon_road.png", 20, 40, 5));
  toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeRelease", "CruiseON:BrakeRelease", "While Driving\nCruise On when radar detected over a certain distance ", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoResumeFromBrakeReleaseDist", "CruiseON:BrakeReleaseDist", "While Driving\nMinimum Cruise On Distance\nDuring long control, it may operate abnormally due to surrounding obstacles.", "../assets/offroad/icon_road.png", 0, 80, 5));
  toggleLayout->addWidget(new ParamControl("AutoResumeFromBrakeReleaseLeadCar", "CruiseON:BrakeReleaseStop", "While Stopping\nCruise On when radar dected within 10M", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("AutoResumeFromGas", "CruiseON:Gas", "Enable Cruise control from Gas", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamControl("AutoCurveSpeedCtrl", "SpeedControl: Vision Curve", "", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoCurveSpeedFactor", "SpeedControl: VisionCurve(50~150%)", "", "../assets/offroad/icon_road.png", 50, 150, 5));
  toggleLayout->addWidget(new ParamControl("AutoNaviSpeedCtrl", "NDA Manager: NAVI speed", "", "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new CValueControl("AutoRoadLimitCtrl", "NDA:RoadLimit(1:Limit,2:Apply)", "", "../assets/offroad/icon_road.png", 0, 2, 1));
  toggleLayout->addWidget(new CValueControl("LongControlActiveSound", "Long Sound 0:OFF,1:Half ON, 2:ON", "", "../assets/offroad/icon_road.png", 0, 2, 1));
#endif
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
