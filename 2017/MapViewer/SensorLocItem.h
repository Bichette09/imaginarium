#pragma once

#include <QGraphicsItem>


class SensorLocItem: public QGraphicsItem
{
public:
	explicit SensorLocItem(QString text, QRectF geometry, QColor pColor, QGraphicsItem *parent=0);
	virtual ~SensorLocItem();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
	QColor color;
	QRectF geometry;
	QString text;
};
