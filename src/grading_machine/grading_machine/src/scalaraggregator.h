#pragma once

// opencv

// std

#include "frame.h"


template <typename T>
class ScalarAggregator
{
	public:
		ScalarAggregator();
		~ScalarAggregator();
		
		void clear();
		void addValue(T pValue);
		
		std::string toString() const;
		
		T	mMin;
		T	mAvg;
		double	mSum;
		T	mMax;
		double	mAggCount;
		
};

#include "scalaraggregator.inl"