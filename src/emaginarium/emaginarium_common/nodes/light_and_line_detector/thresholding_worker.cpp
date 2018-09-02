#include "thresholding_worker.h"

// ros
#include "ros/ros.h"

ThresholdingWorker::Parameters::Parameters()
{
}

ThresholdingWorker::ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters)
	: mCameraThread(NULL)
	, mFrameProviderWorker(pFrameProvider)
	, mParameters(pParameters)
{
	mCameraThread = new FrameProcessor<LightAndLineFrame>(mFrameProviderWorker);
}

ThresholdingWorker::~ThresholdingWorker()
{
	delete mCameraThread; mCameraThread = NULL;
}

bool ThresholdingWorker::computeNextResult(LightAndLineFrame & pRes)
{
	if(!mCameraThread->getNextFrame(pRes))
		return false;
	pRes.setTimestamp(LightAndLineFrame::F_ThresholdingStart);
	
	
	pRes.setTimestamp(LightAndLineFrame::F_ThresholdingDone);
	
	return true;
}