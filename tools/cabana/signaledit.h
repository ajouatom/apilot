#pragma once

#include <QComboBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>

#include "selfdrive/ui/qt/widgets/controls.h"

#include "tools/cabana/canmessages.h"
#include "tools/cabana/dbcmanager.h"

class SignalForm : public QWidget {
public:
  SignalForm(const Signal &sig, QWidget *parent);

  QLineEdit *name, *unit, *comment, *val_desc;
  QSpinBox *size, *offset;
  QDoubleSpinBox *factor, *min_val, *max_val;
  QComboBox *sign, *endianness;
};

class SignalEdit : public QWidget {
  Q_OBJECT

public:
  SignalEdit(int index, const QString &msg_id, const Signal *sig, QWidget *parent = nullptr);
  void setFormVisible(bool show);
  void signalHovered(const Signal *sig);
  inline bool isFormVisible() const { return form_container->isVisible(); }

signals:
  void highlight(const Signal *sig);
  void showChart();
  void showFormClicked();
  void remove(const Signal *sig);
  void save(const Signal *sig, const Signal &new_sig);

protected:
  void enterEvent(QEvent *event) override;
  void leaveEvent(QEvent *event) override;
  void saveSignal();

  SignalForm *form;
  ElidedLabel *title;
  QWidget *form_container;
  QLabel *icon;
  int form_idx = 0;
  QString msg_id;
  const Signal *sig = nullptr;
};

class SignalFindDlg : public QDialog {
  Q_OBJECT

public:
  SignalFindDlg(const QString &id, const Signal *signal, QWidget *parent);
};
