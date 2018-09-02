#pragma once

#include "image_common/frame_processor.h"

// ros

// std



template <typename FrameType>
class FrameProviderWorker : public WorkerInterface<FrameType>
{
public:
	FrameProviderWorker(FrameProvider & pFrameProvider);
	virtual ~FrameProviderWorker();
	
	virtual bool computeNextResult(FrameType & pRes);
	
private:
	FrameProvider & mFrameProvider;
};

#include "frame_provider_worker.inl"