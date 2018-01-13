#include "MyGraphView.h"

#include <QDebug>

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>

using namespace QtCharts;

MyGraphView::MyGraphView(QWidget *pParent)
	: QtCharts::QChartView(pParent)
	, mChart(NULL)
	, mXAxis(NULL)
	, mY1Axis(NULL)
	, mY2Axis(NULL)
{

	this->setRenderHint(QPainter::Antialiasing);

	// create chart
	mChart = new QChart();

	// create axis
	mXAxis = new QValueAxis(mChart);
	mXAxis->setTickCount(2);
	mChart->addAxis(mXAxis, Qt::AlignBottom);

	mY1Axis = new QValueAxis(mChart);
	mY1Axis->setTickCount(5);
	mY1Axis->setMinorTickCount(5);
	mChart->addAxis(mY1Axis, Qt::AlignLeft);

	mY2Axis = new QValueAxis(mChart);
	mY2Axis->setTickCount(5);
	mChart->addAxis(mY2Axis, Qt::AlignRight);


	setChart(mChart);
	setTimeWindow(0.,1.);
}

MyGraphView::~MyGraphView()
{
	mSeries.clear();
	delete mChart; mChart = NULL;
}


MyGraphView::AxisParameters::AxisParameters()
	: mRangeMinValue(0.)
	, mRangeMaxValue(0.)
	, mCenterZero(true)
	, mAutoRange(true)
{

}

void MyGraphView::setAxisParameter(const AxisParameters & pAxisParam, bool pLeftAxis)
{
	QValueAxis * lAxis = pLeftAxis ? mY1Axis : mY2Axis;
	lAxis->setTitleText(QString::fromUtf8(pAxisParam.mLabel.c_str()));

	if(pLeftAxis)
	{
		mLeftAxisParameters = pAxisParam;
	}
	else
	{
		mRightAxisParameters = pAxisParam;
	}

	updateAxisRanges();
}

void MyGraphView::clearValues()
{
	tSeries::iterator lIt = mSeries.begin();
	const tSeries::iterator lItEnd = mSeries.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		lIt.value().first->clear();
	}
	updateAxisRanges();
}

void MyGraphView::declareSerie(const std::string & pSerieName, bool pAttachToLeftAxis, QColor pColor)
{
	QXYSeries * lSerie = new QLineSeries(mChart);
	//lSerie->setUseOpenGL();
	mChart->addSeries(lSerie);
	lSerie->setName(QString::fromUtf8(pSerieName.c_str()));
	lSerie->attachAxis(mXAxis);

	lSerie->attachAxis(pAttachToLeftAxis ? mY1Axis : mY2Axis);
	lSerie->setColor(pColor);
    lSerie->setOpacity(0.6);
	mSeries[pSerieName] = tSeries::mapped_type(lSerie,pAttachToLeftAxis);
}

void MyGraphView::addValue(const std::string & pSerieName, double pX, double pY)
{
	tSeries::iterator lFindIt = mSeries.find(pSerieName);
	if(lFindIt == mSeries.end())
	{
		qWarning()<<"Unknown serie "<<pSerieName.c_str();
		return;
	}
	QXYSeries * lSerie = lFindIt.value().first;
	if(!lSerie)
		return;
	lSerie->append(QPointF(pX,pY));
}

void MyGraphView::setTimeWindow(double pMin, double pMax)
{
	mXAxis->setRange(pMin,pMax);
	mXAxis->setTickCount((int)(pMax - pMin));
	mXAxis->setLabelFormat("");
	updateAxisRanges();
}

void MyGraphView::removeOldest(double pMin)
{
	tSeries::iterator lIt = mSeries.begin();
	const tSeries::iterator lItEnd = mSeries.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		QXYSeries * lSerie = lIt.value().first;
		while(lSerie->count() && lSerie->at(0).x() < pMin)
			lSerie->remove(0);
	}
}



void MyGraphView::updateAxisRanges()
{
	updateAxisRanges(true);
	updateAxisRanges(false);

}

void MyGraphView::updateAxisRanges(bool pLeft)
{
	const AxisParameters & lParam = pLeft ? mLeftAxisParameters : mRightAxisParameters;
	double lMin = 0.;
	double lMax = 1.;
	if(lParam.mAutoRange)
	{

		lMin = std::numeric_limits<double>::max();
		lMax = std::numeric_limits<double>::min();

		tSeries::iterator lIt = mSeries.begin();
		const tSeries::iterator lItEnd = mSeries.end();
		for( ; lIt != lItEnd ; ++lIt)
		{
			if(lIt.value().second != pLeft)
				continue;
			QXYSeries * lSerie = lIt.value().first;

			for(int i = 0 ; i < lSerie->count() ; ++i)
			{
				double lVal = lSerie->at(i).y();
				lMin = std::min(lMin,lVal);
				lMax = std::max(lMax,lVal);
			}
		}

		if(lMin > lMax)
		{
			lMin = lParam.mRangeMinValue;
			lMax = lParam.mRangeMaxValue;
		}
		else
		{
			lMin *= 1.01;
			lMax *= 1.01;
		}

		lMin = std::min(lMin, lParam.mRangeMinValue);
		lMax = std::max(lMax, lParam.mRangeMaxValue);
	}
	else
	{
		lMin = lParam.mRangeMinValue;
		lMax = lParam.mRangeMaxValue;
	}

	if(lParam.mCenterZero)
	{
		double lDelta = std::max(lMax,-lMin);
		lMin = -lDelta;
		lMax = lDelta;
	}

	QValueAxis * lAxis = pLeft ? mY1Axis : mY2Axis;

	lAxis->setRange(lMin,lMax);

}
