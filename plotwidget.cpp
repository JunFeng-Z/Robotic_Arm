#include "plotwidget.h"

#include <QPaintEvent>
#include <QPainter>

PlotWidget::PlotWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PlotWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    painter.fillRect(rect(), QColor("#f4f4f4"));

    const int margin = 24;
    const QRectF area = rect().adjusted(margin, margin, -margin, -margin);

    painter.setPen(QPen(QColor("#d5d5d5"), 1));
    constexpr int gridCount = 4;
    for (int i = 0; i <= gridCount; ++i) {
        const qreal x = area.left() + i * area.width() / gridCount;
        const qreal y = area.top() + i * area.height() / gridCount;
        painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
    }

    painter.setPen(QPen(QColor("#333"), 1));
    painter.drawRect(area);

    auto drawTickLabel = [&painter](const QPointF &pos, const QString &text, Qt::Alignment align) {
        QRectF labelRect(pos.x() - 20, pos.y() - 10, 40, 20);
        painter.drawText(labelRect, align, text);
    };

    const QList<int> labels = {-10, -5, 0, 5, 10};
    for (int i = 0; i < labels.size(); ++i) {
        const qreal t = static_cast<qreal>(i) / (labels.size() - 1);
        const qreal x = area.left() + t * area.width();
        const qreal y = area.bottom() - t * area.height();

        drawTickLabel(QPointF(x, area.bottom() + 14), QString::number(labels[i]), Qt::AlignHCenter | Qt::AlignTop);
        drawTickLabel(QPointF(area.left() - 14, y), QString::number(labels[i]), Qt::AlignRight | Qt::AlignVCenter);
    }

    painter.setPen(QPen(QColor("#9aa9bb"), 2, Qt::DashLine));
    painter.drawLine(QPointF(area.left(), area.center().y()), QPointF(area.right(), area.center().y()));
    painter.drawLine(QPointF(area.center().x(), area.top()), QPointF(area.center().x(), area.bottom()));

    event->accept();
}
