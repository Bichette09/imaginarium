#include "detectionmodel.h"

#include <limits>

DetectionModel::MetricAggregator::MetricAggregator()
{
	clear();
}

DetectionModel::MetricAggregator::~MetricAggregator()
{
}

void DetectionModel::MetricAggregator::clear()
{
	mMin = std::numeric_limits<double>::max();
	mMax = std::numeric_limits<double>::min();
	mAvg = 0.;
	mSum = 0.;
	mAggCount = 0.;
}

void DetectionModel::MetricAggregator::addValue(double pValue)
{
	mMin = std::min(mMin,pValue);
	mMax = std::max(mMax,pValue);
	mSum += pValue;
	++mAggCount;
	mAvg = mSum / mAggCount;
}

std::string DetectionModel::MetricAggregator::toString() const
{
	std::stringstream lSS;
	lSS<<"[ min="<<mMin<<" ; avg="<<mAvg<<" ; max="<<mMax<<" ; var="<< (mAvg == 0. ? std::numeric_limits<double>::infinity() : (mMax-mMin)/mAvg)<<"]";
	return lSS.str();
}

DetectionModel::DetectionModel()
{
	clearModel();
}

DetectionModel::~DetectionModel()
{
}

void DetectionModel::clearModel()
{
	mScalarMetrics.clear();
	mScalarMetrics.resize(SM_Count);
}

void DetectionModel::addAreaToModel(const AreaOfInterest & pArea)
{
	const double lWidth = std::max(pArea.mOBB.size.height,pArea.mOBB.size.width); 
	const double lHeight = std::min(pArea.mOBB.size.height,pArea.mOBB.size.width);
	const double lArea = lWidth * lHeight;
	
	mScalarMetrics[SM_Width].addValue( lWidth );
	mScalarMetrics[SM_Height].addValue( lHeight);
	mScalarMetrics[SM_FillPercentage].addValue( pArea.mPixelCount / lArea);
}

std::string DetectionModel::toString() const
{
	std::stringstream lSS;
	lSS
		<<"width "<<mScalarMetrics[SM_Width].toString()
		<<"\nheight "<<mScalarMetrics[SM_Height].toString()
		<<"\npercentfill "<<mScalarMetrics[SM_FillPercentage].toString();
	
	return lSS.str();
}


bool DetectionModel::testModel(const AreaOfInterest & pArea, float & pScore)
{
	return true;
}
