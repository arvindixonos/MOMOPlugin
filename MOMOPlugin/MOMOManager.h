#pragma once

#include <OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <libfreenect.h>
#include <libfreenect_sync.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "StateMachine.h"
#include "StabilityCheckService.h"

using namespace pcl;

namespace MOMO
{
	class MOMOManager : private boost::noncopyable
	{
	public:
		static boost::shared_ptr<MOMOManager> getInstance();

		void Init();

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

		bool isDeviceStable() const { return deviceStable; }
		void DeviceStable();
		void DeviceUnstable();

		MOMOManager();
		~MOMOManager();

	private:
		static boost::shared_ptr<MOMOManager> instance;

		freenect_context* fnc;
		freenect_device* fnv;

		boost::shared_ptr<StateMachine> momoStateMachine;
		boost::shared_ptr <visualization::PCLVisualizer> pclVisualizer;
		boost::shared_ptr<StabilityCheckService> stabilityCheckService;
		
		double currentDT;

		bool deviceStable;
	
		boost::shared_ptr<openni::Device> openniDevice;
		boost::shared_ptr<pcl::io::OpenNI2Grabber> openni2Grabber;

	};
}
