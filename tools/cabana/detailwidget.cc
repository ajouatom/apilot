#include "tools/cabana/detailwidget.h"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QTimer>

#include "selfdrive/ui/qt/util.h"
#include "tools/cabana/canmessages.h"
#include "tools/cabana/dbcmanager.h"

// DetailWidget

DetailWidget::DetailWidget(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  // tabbar
  tabbar = new QTabBar(this);
  tabbar->setTabsClosable(true);
  tabbar->setDrawBase(false);
  tabbar->setUsesScrollButtons(true);
  tabbar->setAutoHide(true);
  main_layout->addWidget(tabbar);

  // message title
  QFrame *title_frame = new QFrame();
  main_layout->addWidget(title_frame);
  QVBoxLayout *frame_layout = new QVBoxLayout(title_frame);
  title_frame->setFrameShape(QFrame::StyledPanel);
  QHBoxLayout *title_layout = new QHBoxLayout();
  title_layout->addWidget(new QLabel("time:"));
  time_label = new QLabel(this);
  time_label->setStyleSheet("font-weight:bold");
  title_layout->addWidget(time_label);
  title_layout->addStretch();
  name_label = new QLabel(this);
  name_label->setStyleSheet("font-weight:bold;");
  title_layout->addWidget(name_label);
  title_layout->addStretch();
  edit_btn = new QPushButton(tr("Edit"), this);
  edit_btn->setVisible(false);
  title_layout->addWidget(edit_btn);
  frame_layout->addLayout(title_layout);

  // warning
  warning_widget = new QWidget(this);
  QHBoxLayout *warning_hlayout = new QHBoxLayout(warning_widget);
  QLabel *warning_icon = new QLabel(this);
  warning_icon->setPixmap(style()->standardIcon(QStyle::SP_MessageBoxWarning).pixmap({16, 16}));
  warning_hlayout->addWidget(warning_icon);
  warning_label = new QLabel(this);
  warning_hlayout->addWidget(warning_label, 1, Qt::AlignLeft);
  warning_widget->hide();
  frame_layout->addWidget(warning_widget);

  // binary view
  binary_view = new BinaryView(this);
  main_layout->addWidget(binary_view, 0, Qt::AlignTop);

  // signals
  signals_container = new QWidget(this);
  signals_container->setLayout(new QVBoxLayout);
  signals_container->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  scroll = new ScrollArea(this);
  scroll->setWidget(signals_container);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  main_layout->addWidget(scroll);

  // history log
  history_log = new HistoryLog(this);
  main_layout->addWidget(history_log);

  QObject::connect(edit_btn, &QPushButton::clicked, this, &DetailWidget::editMsg);
  QObject::connect(binary_view, &BinaryView::resizeSignal, this, &DetailWidget::resizeSignal);
  QObject::connect(binary_view, &BinaryView::addSignal, this, &DetailWidget::addSignal);
  QObject::connect(can, &CANMessages::updated, this, &DetailWidget::updateState);
  QObject::connect(dbc(), &DBCManager::DBCFileChanged, this, &DetailWidget::dbcMsgChanged);
  QObject::connect(tabbar, &QTabBar::currentChanged, [this](int index) { setMessage(messages[index]); });
  QObject::connect(tabbar, &QTabBar::tabCloseRequested, [=](int index) {
    messages.removeAt(index);
    tabbar->removeTab(index);
    setMessage(messages.isEmpty() ? "" : messages[0]);
  });
}

void DetailWidget::setMessage(const QString &message_id) {
  if (message_id.isEmpty()) return;

  int index = messages.indexOf(message_id);
  if (index == -1) {
    messages.push_back(message_id);
    tabbar->addTab(message_id);
    index = tabbar->count() - 1;
    auto msg = dbc()->msg(message_id);
    tabbar->setTabToolTip(index, msg ? msg->name.c_str() : "untitled");
  }
  tabbar->setCurrentIndex(index);
  msg_id = message_id;
  dbcMsgChanged();
}

void DetailWidget::dbcMsgChanged() {
  if (msg_id.isEmpty()) return;

  warning_widget->hide();
  clearLayout(signals_container->layout());
  QString msg_name = tr("untitled");
  if (auto msg = dbc()->msg(msg_id)) {
    for (int i = 0; i < msg->sigs.size(); ++i) {
      auto form = new SignalEdit(i, msg_id, msg->sigs[i]);
      signals_container->layout()->addWidget(form);
      QObject::connect(form, &SignalEdit::showChart, [this, sig = &msg->sigs[i]]() { emit showChart(msg_id, sig); });
      QObject::connect(form, &SignalEdit::showFormClicked, this, &DetailWidget::showForm);
      QObject::connect(form, &SignalEdit::remove, this, &DetailWidget::removeSignal);
      QObject::connect(form, &SignalEdit::save, this, &DetailWidget::saveSignal);
      QObject::connect(form, &SignalEdit::highlight, binary_view, &BinaryView::highlight);
      QObject::connect(binary_view, &BinaryView::signalHovered, form, &SignalEdit::signalHovered);
    }
    msg_name = msg->name.c_str();
    if (msg->size != can->lastMessage(msg_id).dat.size()) {
      warning_label->setText(tr("Message size (%1) is incorrect!").arg(msg->size));
      warning_widget->show();
    }
  }
  edit_btn->setVisible(true);
  name_label->setText(msg_name);

  binary_view->setMessage(msg_id);
  history_log->setMessage(msg_id);
}

void DetailWidget::updateState() {
  time_label->setText(QString::number(can->currentSec(), 'f', 3));
  if (msg_id.isEmpty()) return;

  binary_view->updateState();
  history_log->updateState();
}

void DetailWidget::showForm() {
  SignalEdit *sender = qobject_cast<SignalEdit *>(QObject::sender());
  for (auto f : signals_container->findChildren<SignalEdit *>()) {
    f->setFormVisible(f == sender && !f->isFormVisible());
    if (f == sender) {
      QTimer::singleShot(0, [=]() { scroll->ensureWidgetVisible(f); });
    }
  }
}

void DetailWidget::editMsg() {
  auto msg = dbc()->msg(msg_id);
  QString name = msg ? msg->name.c_str() : "untitled";
  int size = msg ? msg->size : can->lastMessage(msg_id).dat.size();
  EditMessageDialog dlg(msg_id, name, size, this);
  if (dlg.exec()) {
    dbc()->updateMsg(msg_id, dlg.name_edit->text(), dlg.size_spin->value());
    dbcMsgChanged();
  }
}

void DetailWidget::addSignal(int start_bit, int to) {
  if (dbc()->msg(msg_id)) {
    AddSignalDialog dlg(msg_id, start_bit, to - start_bit + 1, this);
    if (dlg.exec()) {
      dbc()->addSignal(msg_id, dlg.form->getSignal());
      dbcMsgChanged();
    }
  }
}

void DetailWidget::resizeSignal(const Signal *sig, int from, int to) {
  assert(sig != nullptr);
  Signal s = *sig;
  s.start_bit = s.is_little_endian ? from : bigEndianBitIndex(from);;
  s.size = to - from + 1;
  if (s.is_little_endian) {
    s.lsb = s.start_bit;
    s.msb = s.start_bit + s.size - 1;
  } else {
    s.lsb = bigEndianStartBitsIndex(bigEndianBitIndex(s.start_bit) + s.size - 1);
    s.msb = s.start_bit;
  }
  dbc()->updateSignal(msg_id, s.name.c_str(), s);
  dbcMsgChanged();
}

void DetailWidget::saveSignal() {
  SignalEdit *sig_form = qobject_cast<SignalEdit *>(QObject::sender());
  auto s = sig_form->form->getSignal();
  dbc()->updateSignal(msg_id, sig_form->sig_name, s);
  // update binary view and history log
  binary_view->setMessage(msg_id);
  history_log->setMessage(msg_id);
}

void DetailWidget::removeSignal() {
  SignalEdit *sig_form = qobject_cast<SignalEdit *>(QObject::sender());
  QString text = tr("Are you sure you want to remove signal '%1'").arg(sig_form->sig_name);
  if (QMessageBox::Yes == QMessageBox::question(this, tr("Remove signal"), text)) {
    dbc()->removeSignal(msg_id, sig_form->sig_name);
    dbcMsgChanged();
  }
}

// EditMessageDialog

EditMessageDialog::EditMessageDialog(const QString &msg_id, const QString &title, int size, QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Edit message"));
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  QFormLayout *form_layout = new QFormLayout();
  form_layout->addRow("ID", new QLabel(msg_id));

  name_edit = new QLineEdit(title, this);
  form_layout->addRow(tr("Name"), name_edit);

  size_spin = new QSpinBox(this);
  // TODO: limit the maximum?
  size_spin->setMinimum(1);
  size_spin->setValue(size);
  form_layout->addRow(tr("Size"), size_spin);

  main_layout->addLayout(form_layout);

  auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  main_layout->addWidget(buttonBox);
  setFixedWidth(parent->width() * 0.9);

  connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

// ScrollArea

bool ScrollArea::eventFilter(QObject *obj, QEvent *ev) {
  if (obj == widget() && ev->type() == QEvent::Resize) {
    int height = widget()->height() + 4;
    setMinimumHeight(height > 480 ? 480 : height);
    setMaximumHeight(height);
  }
  return QScrollArea::eventFilter(obj, ev);
}

void ScrollArea::setWidget(QWidget *w) {
  QScrollArea::setWidget(w);
  w->installEventFilter(this);
}
