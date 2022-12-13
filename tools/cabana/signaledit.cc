#include "tools/cabana/signaledit.h"

#include <QDoubleValidator>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>

// SignalForm

SignalForm::SignalForm(QWidget *parent) : QWidget(parent) {
  QFormLayout *form_layout = new QFormLayout(this);

  name = new QLineEdit();
  name->setValidator(new QRegExpValidator(QRegExp("^(\\w+)"), name));
  form_layout->addRow(tr("Name"), name);

  size = new QSpinBox();
  size->setMinimum(1);
  form_layout->addRow(tr("Size"), size);

  endianness = new QComboBox();
  endianness->addItems({"Little", "Big"});
  form_layout->addRow(tr("Endianness"), endianness);

  form_layout->addRow(tr("lsb"), lsb = new QLabel());
  form_layout->addRow(tr("msb"), msb = new QLabel());

  sign = new QComboBox();
  sign->addItems({"Signed", "Unsigned"});
  form_layout->addRow(tr("sign"), sign);

  auto double_validator = new QDoubleValidator(this);

  factor = new QLineEdit();
  factor->setValidator(double_validator);
  form_layout->addRow(tr("Factor"), factor);

  offset = new QLineEdit();
  offset->setValidator(double_validator);
  form_layout->addRow(tr("Offset"), offset);

  // TODO: parse the following parameters in opendbc
  unit = new QLineEdit();
  form_layout->addRow(tr("Unit"), unit);
  comment = new QLineEdit();
  form_layout->addRow(tr("Comment"), comment);
  min_val = new QLineEdit();
  min_val->setValidator(double_validator);
  form_layout->addRow(tr("Minimum value"), min_val);
  max_val = new QLineEdit();
  max_val->setValidator(double_validator);
  form_layout->addRow(tr("Maximum value"), max_val);
  val_desc = new QLineEdit();
  form_layout->addRow(tr("Value descriptions"), val_desc);

  QObject::connect(name, &QLineEdit::textEdited, this, &SignalForm::changed);
  QObject::connect(factor, &QLineEdit::textEdited, this, &SignalForm::changed);
  QObject::connect(offset, &QLineEdit::textEdited, this, &SignalForm::changed);
  QObject::connect(sign, SIGNAL(activated(int)), SIGNAL(changed()));
  QObject::connect(endianness, SIGNAL(activated(int)), SIGNAL(changed()));
  QObject::connect(size, SIGNAL(valueChanged(int)), SIGNAL(changed()));
}

// SignalEdit

SignalEdit::SignalEdit(int index, QWidget *parent) : form_idx(index), QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setSpacing(0);

  // title bar
  auto title_bar = new QWidget(this);
  title_bar->setFixedHeight(32);
  QHBoxLayout *title_layout = new QHBoxLayout(title_bar);
  title_layout->setContentsMargins(0, 0, 0, 0);
  title_bar->setStyleSheet("QToolButton {width:15px;height:15px;font-size:15px}");
  color_label = new QLabel(this);
  color_label->setFixedWidth(25);
  color_label->setContentsMargins(5, 0, 0, 0);
  title_layout->addWidget(color_label);
  icon = new QLabel(this);
  title_layout->addWidget(icon);
  title = new ElidedLabel(this);
  title->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  title_layout->addWidget(title);

  plot_btn = new QToolButton(this);
  plot_btn->setText("📈");
  plot_btn->setCheckable(true);
  plot_btn->setAutoRaise(true);
  title_layout->addWidget(plot_btn);
  auto remove_btn = new QToolButton(this);
  remove_btn->setAutoRaise(true);
  remove_btn->setText("x");
  remove_btn->setToolTip(tr("Remove signal"));
  title_layout->addWidget(remove_btn);
  main_layout->addWidget(title_bar);

  // signal form
  form = new SignalForm(this);
  form->setVisible(false);
  main_layout->addWidget(form);

  // bottom line
  QFrame *hline = new QFrame();
  hline->setFrameShape(QFrame::HLine);
  hline->setFrameShadow(QFrame::Sunken);
  main_layout->addWidget(hline);

  save_timer = new QTimer(this);
  save_timer->setInterval(300);
  save_timer->setSingleShot(true);
  save_timer->callOnTimeout(this, &SignalEdit::saveSignal);

  QObject::connect(title, &ElidedLabel::clicked, [this]() { emit showFormClicked(sig); });
  QObject::connect(plot_btn, &QToolButton::clicked, [this](bool checked) {
    emit showChart(msg_id, sig, checked, QGuiApplication::keyboardModifiers() & Qt::ShiftModifier);
  });
  QObject::connect(remove_btn, &QToolButton::clicked,  [this]() { emit remove(sig); });
  QObject::connect(form, &SignalForm::changed, [this]() { save_timer->start(); });
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void SignalEdit::setSignal(const QString &message_id, const Signal *signal) {
  sig = signal;
  updateForm(msg_id == message_id && form->isVisible());
  msg_id = message_id;
  color_label->setText(QString::number(form_idx + 1));
  color_label->setStyleSheet(QString("color:black; background-color:%2").arg(getColor(form_idx)));
  title->setText(sig->name.c_str());
  show();
}

void SignalEdit::saveSignal() {
  if (!sig || !form->changed_by_user) return;

  Signal s = *sig;
  s.name = form->name->text().toStdString();
  s.size = form->size->text().toInt();
  s.offset = form->offset->text().toDouble();
  s.factor = form->factor->text().toDouble();
  s.is_signed = form->sign->currentIndex() == 0;
  bool little_endian = form->endianness->currentIndex() == 0;
  if (little_endian != s.is_little_endian) {
    int start = std::floor(s.start_bit / 8);
    if (little_endian) {
      int end = std::floor((s.start_bit - s.size + 1) / 8);
      s.start_bit = start == end ? s.start_bit - s.size + 1 : bigEndianStartBitsIndex(s.start_bit);
    } else {
      int end = std::floor((s.start_bit + s.size - 1) / 8);
      s.start_bit = start == end ? s.start_bit + s.size - 1 : bigEndianBitIndex(s.start_bit);
    }
    s.is_little_endian = little_endian;
  }
  if (s.is_little_endian) {
    s.lsb = s.start_bit;
    s.msb = s.start_bit + s.size - 1;
  } else {
    s.lsb = bigEndianStartBitsIndex(bigEndianBitIndex(s.start_bit) + s.size - 1);
    s.msb = s.start_bit;
  }
  if (s != *sig)
    emit save(this->sig, s);
}

void SignalEdit::setChartOpened(bool opened) {
  plot_btn->setToolTip(opened ? tr("Close Plot") : tr("Show Plot\nSHIFT click to add to previous opened chart"));
  plot_btn->setChecked(opened);
}

void SignalEdit::updateForm(bool visible) {
  if (visible && sig) {
    form->changed_by_user = false;
    if (form->name->text() != sig->name.c_str()) {
      form->name->setText(sig->name.c_str());
    }
    form->endianness->setCurrentIndex(sig->is_little_endian ? 0 : 1);
    form->sign->setCurrentIndex(sig->is_signed ? 0 : 1);
    form->factor->setText(QString::number(sig->factor));
    form->offset->setText(QString::number(sig->offset));
    form->msb->setText(QString::number(sig->msb));
    form->lsb->setText(QString::number(sig->lsb));
    form->size->setValue(sig->size);
    form->changed_by_user = true;
  }
  form->setVisible(visible);
  icon->setText(visible ? "▼ " : "> ");
}

void SignalEdit::signalHovered(const Signal *s) {
  auto color = sig == s ? "white" : "black";
  color_label->setStyleSheet(QString("color:%1; background-color:%2").arg(color).arg(getColor(form_idx)));
}

void SignalEdit::enterEvent(QEvent *event) {
  emit highlight(sig);
  QWidget::enterEvent(event);
}

void SignalEdit::leaveEvent(QEvent *event) {
  emit highlight(nullptr);
  QWidget::leaveEvent(event);
}
