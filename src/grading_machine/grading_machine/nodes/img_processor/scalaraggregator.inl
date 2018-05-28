
template <typename T>
ScalarAggregator<T>::ScalarAggregator()
{
	clear();
}

template <typename T>
ScalarAggregator<T>::~ScalarAggregator()
{
}

template <typename T>
void ScalarAggregator<T>::clear()
{
	mMin = std::numeric_limits<T>::max();
	mMax = std::numeric_limits<T>::min();
	mAvg = std::numeric_limits<T>::infinity();
	mSum = 0.;
	mAggCount = 0.;
}

template <typename T>
void ScalarAggregator<T>::addValue(T pValue)
{
	mMin = std::min(mMin,pValue);
	mMax = std::max(mMax,pValue);
	mSum += pValue;
	++mAggCount;
	mAvg = static_cast<T>(mSum / mAggCount);
}

template <typename T>
std::string ScalarAggregator<T>::toString() const
{
	std::stringstream lSS;
	lSS<<"[ min="<<mMin<<" ; avg="<<mAvg<<" ; max="<<mMax<<" ; var="<< (mAvg == 0. ? std::numeric_limits<double>::infinity() : (mMax-mMin)/mAvg)<<"]";
	return lSS.str();
}