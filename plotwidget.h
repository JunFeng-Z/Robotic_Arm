#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
};

#endif // PLOTWIDGET_H
