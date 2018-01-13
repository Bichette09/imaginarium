#include "SensorLocItem.h"

#include <QPainter>

SensorLocItem::SensorLocItem(QString text, QRectF geometry, QColor pColor, QGraphicsItem *parent)
		: QGraphicsItem(parent)
		, text(text)
		, geometry(geometry)
		, color(pColor)
{

}

SensorLocItem::~SensorLocItem()
{
}

QRectF SensorLocItem::boundingRect() const
{
	return geometry;
}

void SensorLocItem::paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/)
{
	painter->save();
	painter->setBrush(QBrush(Qt::darkGray,Qt::SolidPattern));
	painter->setPen(Qt::transparent);
	painter->setFont(QFont("Arial",4));
	painter->drawRoundedRect(QRectF(geometry.center() - QPointF(3.,3.),geometry.center() + QPointF(3.,3.)),4.,4.);
	painter->setPen(color);
	painter->drawText(geometry, text, Qt::AlignCenter | Qt::AlignVCenter);
	painter->restore();
}
