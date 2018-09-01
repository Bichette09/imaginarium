template <typename FrameType>
FrameProviderWorker<FrameType>::FrameProviderWorker(FrameProvider & pFrameProvider)
	: mFrameProvider(pFrameProvider)
{
}

template <typename FrameType>
FrameProviderWorker<FrameType>::~FrameProviderWorker()
{
}

template <typename FrameType>
bool FrameProviderWorker<FrameType>::computeNextResult(FrameType & pRes)
{
	return mFrameProvider.getNextFrame(pRes);
}
