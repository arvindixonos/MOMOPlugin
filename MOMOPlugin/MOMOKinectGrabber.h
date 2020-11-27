#pragma once


#include <OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/io/image.h>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>


using namespace pcl::io;

namespace MOMO
{
	class MOMOKinectGrabber : public OpenNI2Grabber
	{
	public:
		MOMOKinectGrabber(const std::string& device_id = "",
			const Mode& depth_mode = OpenNI_Default_Mode,
			const Mode& image_mode = OpenNI_Default_Mode);
		~MOMOKinectGrabber();

	private:

	};
}