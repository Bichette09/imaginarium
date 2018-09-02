#include "frame_interface.h"

FrameInterface::FrameInterface()
{
}

FrameInterface::~FrameInterface()
{
}

void FrameInterface::ensureSizeAndType(int pWidth, int pHeight)
{
	// ensure that mat have the right format
	cv::Mat & lY = editY();
	if(lY.type() != CV_8UC1 || lY.size[1] != pWidth || lY.size[0] != pHeight)
	{
		lY = cv::Mat(pHeight,pWidth,CV_8UC1);
	}
	cv::Mat & lU = editU();
	if(lU.type() != CV_8UC1 || lU.size[1] != pWidth || lU.size[0] != pHeight)
	{
		lU = cv::Mat(pHeight,pWidth,CV_8UC1);
	}
	cv::Mat & lV = editV();
	if(lV.type() != CV_8UC1 || lV.size[1] != pWidth || lV.size[0] != pHeight)
	{
		lV = cv::Mat(pHeight,pWidth,CV_8UC1);
	}
}