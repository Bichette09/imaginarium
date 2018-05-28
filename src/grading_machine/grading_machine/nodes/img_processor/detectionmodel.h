#pragma once

// opencv

// std

#include "frame.h"
#include "scalaraggregator.h"



class DetectionModel
{
	public:
		DetectionModel();
		~DetectionModel();
		
		void clearModel();
		void addAreaToModel(const AreaOfInterest & pArea);
		bool testModel(const AreaOfInterest & pArea, float & pScore);
		
		std::string toString() const;
		
	private:
		enum ScalarMetrics
		{
			/** longest part of OBB
			*/
			SM_Width = 0,
			/** smallest part of OBB
			*/
			SM_Height,
			/** how many pixels are valid in OBB
			*/
			SM_FillPercentage,
			
			
			SM_Count
		};
		

		
		std::vector< ScalarAggregator<double> > mScalarMetrics;
};