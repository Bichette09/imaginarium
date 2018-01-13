#pragma once

#include <QtCharts/QChartView>

namespace QtCharts {
	class QChart;
	class QXYSeries;
	class QValueAxis;
}

class MyGraphView : public QtCharts::QChartView
{
	Q_OBJECT;
public:
	MyGraphView(QWidget * pParent = NULL);
	virtual ~MyGraphView();

	struct AxisParameters
	{
		AxisParameters();

		std::string		mLabel;
		double			mRangeMinValue;
		double			mRangeMaxValue;
		bool			mCenterZero;
		bool			mAutoRange;
	};

	void clearValues();

	void setAxisParameter(const AxisParameters & pAxisParam, bool pLeftAxis);

	void declareSerie(const std::string & pSerieName, bool pAttachToLeftAxis, QColor pColor);
	void addValue(const std::string & pSerieName, double pX, double pY);
	void setTimeWindow(double pMin, double pMax);
	void removeOldest(double pMin);

	void updateAxisRanges();


private:
	void updateAxisRanges(bool pLeft);
	typedef QMap<std::string, std::pair< QtCharts::QXYSeries *,bool> > tSeries;

	AxisParameters				mLeftAxisParameters;
	AxisParameters				mRightAxisParameters;
	QtCharts::QChart *			mChart;
	QtCharts::QValueAxis *		mXAxis;
	QtCharts::QValueAxis *		mY1Axis;
	QtCharts::QValueAxis *		mY2Axis;
	tSeries						mSeries;
};
