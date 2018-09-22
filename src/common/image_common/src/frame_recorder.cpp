#include "frame_recorder.h"

#include "image_common/frame_interface.h"




FrameRecorder::FrameRecorder(const std::string & pOutputFolder,int pWidth, int pHeight, int pFps, ros::NodeHandle & pNode)
	: mOutputFolder(pOutputFolder)
	, mVideoRecorder(NULL)
	, mWidth(pWidth)
	, mHeight(pHeight)
	, mFps(pFps)
{
	mGamePadSubscriber = pNode.subscribe("/GamePadButtons", 1000, &FrameRecorder::onGamepadButton,this);
	
}

FrameRecorder::~FrameRecorder()
{
	endRecording();
}

void FrameRecorder::onGamepadButton(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data.find("|VideoRec_Start|") != std::string::npos)
	{
		startRecording();
	}
	else if(msg->data.find("|VideoRec_Stop|")  != std::string::npos)
	{
		endRecording();
	}
}

void FrameRecorder::startRecording()
{
	endRecording();
	
	std::string lOutputFile;
	
	for(int i = 0 ; i < 64 ; ++i)
	{
		std::stringstream lSS;
		lSS<<mOutputFolder<<"/vid_"<<i<<".avi";
		lOutputFile = lSS.str();
		if( access(lOutputFile.c_str(),F_OK) == -1)
			break;
		lOutputFile.clear();
	}
	
	if(lOutputFile.empty())
	{
		ROS_ERROR_STREAM("Too many videos");
		return;
	}

	mVideoRecorder = new cv::VideoWriter(lOutputFile,CV_FOURCC('M','J','P','G'),mFps,cv::Size(mWidth,mHeight));
	if(!mVideoRecorder->isOpened())
	{
		ROS_ERROR_STREAM("Fail to start recording");
		delete mVideoRecorder; mVideoRecorder = NULL;
		return;
	}
	
	ROS_WARN_STREAM("Start recording in "<<lOutputFile);
	
}

void FrameRecorder::addFrame( FrameInterface & pFrame)
{
	if(!mVideoRecorder)
		return;
	std::vector<cv::Mat> lToMerge;
	lToMerge.push_back(pFrame.editY());
	lToMerge.push_back(pFrame.editU());
	lToMerge.push_back(pFrame.editV());
	cv::merge(lToMerge, mTmp);
	cvtColor(mTmp, mTmp2, cv::COLOR_YUV2BGR);
	mVideoRecorder->write(mTmp2);
	
}

void FrameRecorder::endRecording()
{
	if(!mVideoRecorder)
		return;
	mVideoRecorder->release();
	delete mVideoRecorder; mVideoRecorder = NULL;
	
	ROS_WARN_STREAM("End recording");
	
}
