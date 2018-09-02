#pragma once

#include "light_and_line_frame.h"
#include "image_common/frame_processor.h"
#include "image_common/frame_provider_worker.h"

class ThresholdingWorker : public WorkerInterface<LightAndLineFrame>
{
public:

	struct Parameters
	{
		Parameters();
		
		
	};
	
	ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters);
	virtual ~ThresholdingWorker();
	
	const Parameters &			mParameters;
	
protected:
	virtual bool computeNextResult(LightAndLineFrame & pRes);
private:
	FrameProviderWorker<LightAndLineFrame>	mFrameProviderWorker;
	FrameProcessor<LightAndLineFrame> *		mCameraThread;
	
};