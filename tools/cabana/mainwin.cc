#include "tools/cabana/mainwin.h"

#include <QApplication>
#include <QCompleter>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScreen>
#include <QVBoxLayout>

#include "tools/replay/util.h"

static MainWindow *main_win = nullptr;
void qLogMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
  if (main_win) emit main_win->showMessage(msg, 0);
}

MainWindow::MainWindow() : QWidget() {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(11, 11, 11, 5);
  main_layout->setSpacing(0);

  QHBoxLayout *h_layout = new QHBoxLayout();
  h_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->addLayout(h_layout);

  splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setHandleWidth(11);

  // DBC file selector
  QWidget *messages_container = new QWidget(this);
  QVBoxLayout *messages_layout = new QVBoxLayout(messages_container);
  messages_layout->setContentsMargins(0, 0, 0, 0);
  QHBoxLayout *dbc_file_layout = new QHBoxLayout();
  dbc_combo = new QComboBox(this);
  auto dbc_names = dbc()->allDBCNames();
  for (const auto &name : dbc_names) {
    dbc_combo->addItem(QString::fromStdString(name));
  }
  dbc_combo->model()->sort(0);
  dbc_combo->setEditable(true);
  dbc_combo->setCurrentText(QString());
  dbc_combo->setInsertPolicy(QComboBox::NoInsert);
  dbc_combo->completer()->setCompletionMode(QCompleter::PopupCompletion);
  QFont font;
  font.setBold(true);
  dbc_combo->lineEdit()->setFont(font);
  dbc_file_layout->addWidget(dbc_combo);

  QPushButton *load_from_paste = new QPushButton(tr("Load from paste"), this);
  dbc_file_layout->addWidget(load_from_paste);

  dbc_file_layout->addStretch();
  QPushButton *save_btn = new QPushButton(tr("Save DBC"), this);
  dbc_file_layout->addWidget(save_btn);
  messages_layout->addLayout(dbc_file_layout);

  messages_widget = new MessagesWidget(this);
  messages_layout->addWidget(messages_widget);
  splitter->addWidget(messages_container);

  charts_widget = new ChartsWidget(this);
  detail_widget = new DetailWidget(charts_widget, this);
  splitter->addWidget(detail_widget);

  h_layout->addWidget(splitter);

  // right widgets
  QWidget *right_container = new QWidget(this);
  right_container->setFixedWidth(640);
  r_layout = new QVBoxLayout(right_container);
  r_layout->setContentsMargins(11, 0, 0, 0);
  QHBoxLayout *right_hlayout = new QHBoxLayout();
  QLabel *fingerprint_label = new QLabel(this);
  right_hlayout->addWidget(fingerprint_label);

  // TODO: click to select another route.
  right_hlayout->addWidget(new QLabel(can->route()));
  QPushButton *settings_btn = new QPushButton("Settings");
  right_hlayout->addWidget(settings_btn, 0, Qt::AlignRight);

  r_layout->addLayout(right_hlayout);

  video_widget = new VideoWidget(this);
  r_layout->addWidget(video_widget, 0, Qt::AlignTop);

  r_layout->addWidget(charts_widget);

  h_layout->addWidget(right_container);

  // status bar
  status_bar = new QStatusBar(this);
  status_bar->setFixedHeight(20);
  status_bar->setContentsMargins(0, 0, 0, 0);
  status_bar->setSizeGripEnabled(true);
  progress_bar = new QProgressBar();
  progress_bar->setRange(0, 100);
  progress_bar->setTextVisible(true);
  progress_bar->setFixedSize({230, 16});
  progress_bar->setVisible(false);
  status_bar->addPermanentWidget(progress_bar);
  main_layout->addWidget(status_bar);

  qRegisterMetaType<uint64_t>("uint64_t");
  qRegisterMetaType<ReplyMsgType>("ReplyMsgType");
  installMessageHandler([this](ReplyMsgType type, const std::string msg) {
    // use queued connection to recv the log messages from replay.
    emit showMessage(QString::fromStdString(msg), 3000);
  });
  installDownloadProgressHandler([this](uint64_t cur, uint64_t total, bool success) {
    emit updateProgressBar(cur, total, success);
  });

  main_win = this;
  qInstallMessageHandler(qLogMessageHandler);
  QFile json_file("./car_fingerprint_to_dbc.json");
  if (json_file.open(QIODevice::ReadOnly)) {
    fingerprint_to_dbc = QJsonDocument::fromJson(json_file.readAll());
  }

  QObject::connect(dbc_combo, SIGNAL(activated(const QString &)), SLOT(loadDBCFromName(const QString &)));
  QObject::connect(load_from_paste, &QPushButton::clicked, this, &MainWindow::loadDBCFromPaste);
  QObject::connect(save_btn, &QPushButton::clicked, this, &MainWindow::saveDBC);
  QObject::connect(this, &MainWindow::showMessage, status_bar, &QStatusBar::showMessage);
  QObject::connect(this, &MainWindow::updateProgressBar, this, &MainWindow::updateDownloadProgress);
  QObject::connect(messages_widget, &MessagesWidget::msgSelectionChanged, detail_widget, &DetailWidget::setMessage);
  QObject::connect(charts_widget, &ChartsWidget::dock, this, &MainWindow::dockCharts);
  QObject::connect(charts_widget, &ChartsWidget::rangeChanged, video_widget, &VideoWidget::rangeChanged);
  QObject::connect(settings_btn, &QPushButton::clicked, this, &MainWindow::setOption);
  QObject::connect(can, &CANMessages::streamStarted, this, &MainWindow::loadDBCFromFingerprint);
  QObject::connect(can, &CANMessages::streamStarted, [=]() { fingerprint_label->setText(can->carFingerprint() ); });
}

void MainWindow::loadDBCFromName(const QString &name) {
  if (name != dbc()->name()) {
    dbc()->open(name);
    dbc_combo->setCurrentText(name);
  }
}

void MainWindow::loadDBCFromPaste() {
  LoadDBCDialog dlg(this);
  if (dlg.exec()) {
    dbc()->open("from paste", dlg.dbc_edit->toPlainText());
    dbc_combo->setCurrentText("loaded from paste");
  }
}

void MainWindow::loadDBCFromFingerprint() {
  auto fingerprint = can->carFingerprint();
  if (!fingerprint.isEmpty() && dbc()->name().isEmpty()) {
    auto dbc_name = fingerprint_to_dbc[fingerprint];
    if (dbc_name != QJsonValue::Undefined) {
      loadDBCFromName(dbc_name.toString());
    }
  }
}

void MainWindow::saveDBC() {
  SaveDBCDialog dlg(this);
  dlg.dbc_edit->setText(dbc()->generateDBC());
  dlg.exec();
}

void MainWindow::updateDownloadProgress(uint64_t cur, uint64_t total, bool success) {
   if (success && cur < total) {
    progress_bar->setValue((cur / (double)total) * 100);
    progress_bar->setFormat(tr("Downloading %p% (%1)").arg(formattedDataSize(total).c_str()));
    progress_bar->show();
  } else {
    progress_bar->hide();
  }
}

void MainWindow::dockCharts(bool dock) {
  if (dock && floating_window) {
    floating_window->removeEventFilter(charts_widget);
    r_layout->addWidget(charts_widget);
    floating_window->deleteLater();
    floating_window = nullptr;
  } else if (!dock && !floating_window) {
    floating_window = new QWidget(nullptr);
    floating_window->setLayout(new QVBoxLayout());
    floating_window->layout()->addWidget(charts_widget);
    floating_window->installEventFilter(charts_widget);
    floating_window->setMinimumSize(QGuiApplication::primaryScreen()->size() / 2);
    floating_window->showMaximized();
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  main_win = nullptr;
  if (floating_window)
    floating_window->deleteLater();
  QWidget::closeEvent(event);
}

void MainWindow::setOption() {
  SettingsDlg dlg(this);
  dlg.exec();
}

// LoadDBCDialog

LoadDBCDialog::LoadDBCDialog(QWidget *parent) : QDialog(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  dbc_edit = new QTextEdit(this);
  dbc_edit->setAcceptRichText(false);
  dbc_edit->setPlaceholderText(tr("paste DBC file here"));
  main_layout->addWidget(dbc_edit);
  auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  main_layout->addWidget(buttonBox);

  setMinimumSize({640, 480});
  QObject::connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
  QObject::connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

// SaveDBCDialog

SaveDBCDialog::SaveDBCDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Save DBC"));
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  dbc_edit = new QTextEdit(this);
  dbc_edit->setAcceptRichText(false);
  main_layout->addWidget(dbc_edit);

  QPushButton *copy_to_clipboard = new QPushButton(tr("Copy To Clipboard"), this);
  QPushButton *save_as = new QPushButton(tr("Save As"), this);

  QHBoxLayout *btn_layout = new QHBoxLayout();
  btn_layout->addStretch();
  btn_layout->addWidget(copy_to_clipboard);
  btn_layout->addWidget(save_as);
  main_layout->addLayout(btn_layout);
  setMinimumSize({640, 480});

  QObject::connect(copy_to_clipboard, &QPushButton::clicked, this, &SaveDBCDialog::copytoClipboard);
  QObject::connect(save_as, &QPushButton::clicked, this, &SaveDBCDialog::saveAs);
}

void SaveDBCDialog::copytoClipboard() {
  dbc_edit->selectAll();
  dbc_edit->copy();
  QDialog::accept();
}

void SaveDBCDialog::saveAs() {
  QString file_name = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                   QDir::homePath() + "/untitled.dbc", tr("DBC (*.dbc)"));
  if (!file_name.isEmpty()) {
    QFile file(file_name);
    if (file.open(QIODevice::WriteOnly)) {
      file.write(dbc_edit->toPlainText().toUtf8());
    }
    QDialog::accept();
  }
}
