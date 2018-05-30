#include "detectionmodel.h"

#include <limits>


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
