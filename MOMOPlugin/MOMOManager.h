#pragma once

#include <OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/io/image.h>
#include <boost/array.hpp>
//#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <libfreenect.h>
#include <libfreenect_sync.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "StateMachine.h"
#include "StabilityCheckService.h"
#include "CalibrationService.h"

using namespace pcl;
using namespace pcl::io::openni2;

namespace MOMO
{
	class MOMOManager : private boost::noncopyable
	{
	public:
		static boost::shared_ptr<MOMOManager> getInstance();

		void Init();

		void InitBuffers();

		void SwitchToCalibrationMode();
		void SwitchToGameMode();

		void InitOpenNIGrabber();

		void AddDeviceStates();

		void InitServices();

		void InitStateMachine();

		void InitVisualizer();

		void Initfreenect();

		void Update(double dt);
		void Exit();
		bool isStopped();

		void OnDeviceStable();
		void OnDeviceUnstable();

		void UpdateTiltState();

		boost::array<double, 3> GetAccelerometerValues();

		double getDT() const { return currentDT; }

		void CloudAvailable(const PointCloud<PointXYZ>::ConstPtr &cloud);
		void ColorImageAvailable(const boost::shared_ptr<pcl::io::openni2::Image>& colorImage);
		void DepthImageAvailable(const boost::shared_ptr<pcl::io::openni2::DepthImage>& depthImage);

		bool isDeviceStable() const { return deviceStable; }
		void DeviceStable();
		void DeviceUnstable();
	
		bool isCalibrating() { return calibrationMode; }

		unsigned char* GetRGBData() const { return rgbData; }
		cv::Mat GetRGBMat();

		unsigned short* GetDepthData() const { return depthData; }
		cv::Mat GetDepthMat() const { return depthMat; }

		cv::Point3f GetWorldPosition(int depthX, int depthY, int depthZ);

		MOMOManager();
		~MOMOManager();
	private:
		static boost::shared_ptr<MOMOManager> instance;

		freenect_context* fnc;
		freenect_device* fnv;

		boost::shared_ptr<StateMachine> momoStateMachine;
		boost::shared_ptr <visualization::PCLVisualizer> pclVisualizer;

		boost::shared_ptr<StabilityCheckService> stabilityCheckService;
		boost::shared_ptr<CalibrationService> calibrationService;

		double currentDT;

		bool deviceStable;
	
		boost::shared_ptr<openni::Device> openniDevice;
		boost::shared_ptr<pcl::io::OpenNI2Grabber> openni2Grabber;
		boost::shared_ptr<openni::VideoStream> depthStream;

		bool calibrationMode;

		boost::mutex colorImageMutex;
		boost::shared_ptr<pcl::io::openni2::Image> colorImageBuffer;
		unsigned char* rgbData;
		cv::Mat rgbMat;
		int rgbDataSize;

		boost::mutex depthImageMutex;
		boost::shared_ptr<pcl::io::openni2::DepthImage> depthImageBuffer;
		unsigned short* depthData;
		cv::Mat depthMat;
		int depthDataSize;

		boost::mutex cloudMutex;
		PointCloud<PointXYZ>::ConstPtr cloudBuffer;

	};
}
