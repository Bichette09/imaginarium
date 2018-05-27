#pragma once

// opencv

// std

#include "frame.h"




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
	
		class MetricAggregator
		{
			public:
				MetricAggregator();
				~MetricAggregator();
				
				void clear();
				void addValue(double pValue);
				
				std::string toString() const;
				
				double	mMin;
				double	mAvg;
				double	mSum;
				double	mMax;
				double	mAggCount;
				
		};

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
		std::vector<MetricAggregator> mScalarMetrics;
};