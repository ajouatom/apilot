#pragma once

#include <QList>
#include <QSet>
#include <QStyledItemDelegate>
#include <QTableView>

#include "tools/cabana/dbcmanager.h"

class BinaryItemDelegate : public QStyledItemDelegate {
  Q_OBJECT

public:
  BinaryItemDelegate(QObject *parent);
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setSelectionColor(const QColor &color) { selection_color = color; }

private:
  QFont small_font, hex_font;
  QColor selection_color;
};

class BinaryViewModel : public QAbstractTableModel {
  Q_OBJECT

public:
  BinaryViewModel(QObject *parent) : QAbstractTableModel(parent) {}
  void setMessage(const QString &message_id);
  void updateState();
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const { return {}; }
  int rowCount(const QModelIndex &parent = QModelIndex()) const override { return row_count; }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override { return column_count; }
  QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override {
    return createIndex(row, column, (void *)&items[row * column_count + column]);
  }
  Qt::ItemFlags flags(const QModelIndex &index) const override {
    return (index.column() == column_count - 1) ? Qt::ItemIsEnabled : Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  }

  struct Item {
    QColor bg_color = QColor(Qt::white);
    bool is_msb = false;
    bool is_lsb = false;
    QString val = "0";
    QList<const Signal *> sigs;
  };
  std::vector<Item> items;

private:
  QString msg_id;
  const DBCMsg *dbc_msg;
  int row_count = 0;
  const int column_count = 9;
};

class BinaryView : public QTableView {
  Q_OBJECT

public:
  BinaryView(QWidget *parent = nullptr);
  void setMessage(const QString &message_id);
  void highlight(const Signal *sig);
  QSet<const Signal*> getOverlappingSignals() const;
  inline const Signal *hoveredSignal() const { return hovered_sig; }
  inline void updateState() { model->updateState(); }

signals:
  void signalHovered(const Signal *sig);
  void addSignal(int start_bit, int size, bool little_endian);
  void resizeSignal(const Signal *sig, int from, int size);

private:
  void setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags flags) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void leaveEvent(QEvent *event) override;

  QModelIndex anchor_index;
  BinaryViewModel *model;
  BinaryItemDelegate *delegate;
  const Signal *resize_sig = nullptr;
  const Signal *hovered_sig = nullptr;
};
