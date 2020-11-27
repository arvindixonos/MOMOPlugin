#include "MOMOKinectGrabber.h"

MOMO::MOMOKinectGrabber::MOMOKinectGrabber(const std::string& device_id = "",
	const Mode& depth_mode = OpenNI_Default_Mode,
	const Mode& image_mode = OpenNI_Default_Mode)

	:OpenNI2Grabber(device_id, depth_mode, image_mode)
{
}

MOMO::MOMOKinectGrabber::~MOMOKinectGrabber()
{

}
