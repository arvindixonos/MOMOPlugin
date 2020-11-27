#ifdef BOOST_OS_WINDOWS
#define _WIN32_WINNT 0x0501
#if _WIN32_WINNT <= 0x0501
#define BOOST_ASIO_DISABLE_IOCP
#define BOOST_ASIO_ENABLE_CANCELIO
#endif
#endif

#include <tinyxml2.h>


#include "MOMOManager.h"

//using namespace openni;
//using namespace nite;
//using namespace pcl;
//using namespace pcl::io::openni2;
//using namespace boost;

using namespace MOMO;
using namespace tinyxml2;


// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
#define FREENECT_COUNTS_PER_G 819.
#define GRAVITY 9.80665

boost::shared_ptr<MOMOManager> momoManager = nullptr;
double currentDt = 0.0;
//
//boost::shared_ptr<Device> openniDevice;
//boost::shared_ptr<pcl::io::OpenNI2Grabber> openni2Grabber;

//SACSegmentation<PointXYZ> seg;
//ModelCoefficients::Ptr coefficients(new ModelCoefficients);
//PointIndices::Ptr inlierIndices(new PointIndices);
//
//boost::mutex cloudReadMutex;
//
//PointCloud<PointXYZ>::ConstPtr cloudBuffer;
//PointCloud<PointXYZ>::ConstPtr rawCloud;
//PointCloud<PointXYZ>::Ptr filteredCloud(new PointCloud<PointXYZ>);
//PointCloud<PointXYZ>::Ptr segmentedCloud(new PointCloud<PointXYZ>);
//PointCloud<PointXYZ>::Ptr wallRemovedCloud(new PointCloud<PointXYZ>);
//visualization::PointCloudColorHandlerCustom<PointXYZ> segmentedCloudColor(segmentedCloud, 255, 0, 0);
//visualization::PointCloudColorHandlerCustom<PointXYZ> wallRemovedCloudColor(wallRemovedCloud, 255, 255, 255);
//
//VoxelGrid<pcl::PointXYZ> voxelFilter;
//ExtractIndices<pcl::PointXYZ> pointsExtractor;
//
//HandTracker handTracker;
//
//bool shouldExit = false;
//int framecounter = 0;
//
//visualization::PCLVisualizer pclVisualizer("PCL Visualizer");


//
///// Enumeration of tilt motor status
//typedef enum {
//	TILT_STATUS_STOPPED = 0x00, /**< Tilt motor is stopped */
//	TILT_STATUS_LIMIT = 0x01, /**< Tilt motor has reached movement limit */
//	TILT_STATUS_MOVING = 0x04, /**< Tilt motor is currently moving to new position */
//} tilt_status_code;
//
//
///// Data from the tilt motor and accelerometer
//typedef struct {
//	float                   accelerometer_x; /**< Raw accelerometer data for X-axis, see FREENECT_COUNTS_PER_G for conversion */
//	float                   accelerometer_y; /**< Raw accelerometer data for Y-axis, see FREENECT_COUNTS_PER_G for conversion */
//	float                   accelerometer_z; /**< Raw accelerometer data for Z-axis, see FREENECT_COUNTS_PER_G for conversion */
//	int						tilt_angle;      /**< Raw tilt motor angle encoder information */
//	tilt_status_code		tilt_status;     /**< State of the tilt motor (stopped, moving, etc...) */
//} raw_tilt_state;
//
//
//libusb_device_handle *dev;
//raw_tilt_state tilt_state;
//libusb_context* ctx;

//void cloud_cb_(const PointCloud<PointXYZ>::ConstPtr &cloud)
//{
//	/*boost::mutex::scoped_lock lock(cloudReadMutex);
//	cloudBuffer = cloud->makeShared();
//
//	framecounter++;*/
//	//std::cout << "Got Frame: "<<framecounter<<"\n";
//}

int Setup()
{
	momoManager = MOMOManager::getInstance();
	momoManager->Init();

	//freenect_context *f_ctx;
	//freenect_device *f_dev;
	//int res;

	//if (freenect_init(&f_ctx, NULL) < 0) {
	//	printf("freenect_init() failed\n");
	//	return 1;
	//}

	/*
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices(f_ctx);
	printf("Number of devices found: %d\n", nr_devices);

	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	freenect_sync_set_tilt_degs(0, 0);*/

	/*openni::Status opennistatus = OpenNI::initialize();

	openniDevice = boost::make_shared<openni::Device>();

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	opennistatus = openniDevice->open(ANY_DEVICE);

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Unable to open device");
		return 1;
	}	

	const std::string device_id = openniDevice->getDeviceInfo().getUri();
	openni2Grabber = boost::make_shared<pcl::io::OpenNI2Grabber>(device_id);

	
	boost::function<void(const PointCloud<PointXYZ>::ConstPtr&)> f = boost::bind(cloud_cb_, _1);*/
	
	/*
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setDistanceThreshold(0.1);

	pclVisualizer.setBackgroundColor(0.02, 0.02, 0.02);
	pclVisualizer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");

	openni2Grabber->registerCallback(f);
	openni2Grabber->start();*/

	/*nite::Status nitestatus = NiTE::initialize();
	nitestatus = handTracker.create();
	handTracker.startGestureDetection(GestureType::GESTURE_WAVE);*/


	//libusb_init(&ctx);

	//libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	//ssize_t cnt = libusb_get_device_list(ctx, &devs); //get the list of devices

	//for (int i = 0; i < cnt; ++i)
	//{
	//	struct libusb_device_descriptor desc;
	//	const int r = libusb_get_device_descriptor(devs[i], &desc);
	//	if (r < 0)
	//		continue;

	//	printf("Device: %i Vendor: %i Product: %i\n", i, desc.idVendor, desc.idProduct);

	//	if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
	//	{
	//		if ((libusb_open(devs[i], &dev) != 0) || (dev == 0))
	//		{
	//			printf("Cannot open device!");
	//			break;
	//		}

	//		printf("Opening Device");
	//		libusb_claim_interface(dev, 0);
	//		break;
	//	}
	//}

	//libusb_free_device_list(devs, 1);

 	return 0;
}

void Update()
{
	momoManager->Update(currentDt);
}

void Exit()
{
	/*handTracker.stopGestureDetection(GestureType::GESTURE_WAVE);
	handTracker.destroy();

	openni2Grabber->stop();
	openniDevice->close();

	nite::NiTE::shutdown();

	openni::OpenNI::shutdown();*/

	momoManager->Exit();
}


void deviceupdate()
{
	//freenect_context *ctx = dev->parent;
	//uint8_t buf[10];
	//uint16_t ux, uy, uz;

	//int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 10);
	//if (ret != 10) {
	//	printf("Error in accelerometer reading, libusb_control_transfer returned %d\n", ret);
	//	return;// ret < 0 ? ret : -1;
	//}

	//ux = ((uint16_t)buf[2] << 8) | buf[3];
	//uy = ((uint16_t)buf[4] << 8) | buf[5];
	//uz = ((uint16_t)buf[6] << 8) | buf[7];

	//tilt_state.accelerometer_x = (int16_t)ux / FREENECT_COUNTS_PER_G * GRAVITY;
	//tilt_state.accelerometer_y = (int16_t)uy / FREENECT_COUNTS_PER_G * GRAVITY;
	//tilt_state.accelerometer_z = (int16_t)uz / FREENECT_COUNTS_PER_G * GRAVITY;
	//tilt_state.tilt_angle = (int)buf[8];
	//tilt_state.tilt_status = (tilt_status_code)buf[9];
}

int main()
{
	Setup();

	boost::chrono::steady_clock::time_point startTime = boost::chrono::steady_clock::now();

	while (!momoManager->isStopped()) 
	{
		boost::chrono::steady_clock::time_point endTime = boost::chrono::steady_clock::now();
		boost::chrono::duration<double> sec = endTime - startTime;
		
		currentDt = sec.count();

		startTime = endTime;

	//	//deviceupdate();
	//	//int tiltAngle = tilt_state.tilt_angle / 2;
	//	//printf("Tilt Acc: %3f %3f %3f \n", tilt_state.accelerometer_x, tilt_state.accelerometer_y, tilt_state.accelerometer_z);

	//	//nite::HandTrackerFrameRef handTrackerFrame;
	//	//nite::Status niteStatus = handTracker.readFrame(&handTrackerFrame);

	//	//const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
	//	//for (int i = 0; i < gestures.getSize(); ++i)
	//	//{
	//	//	std::cout << "Gesture detected IN \n";

	//	//	if (gestures[i].isComplete())
	//	//	{
	//	//		std::cout << "Gesture Complete \n";

	//	//		nite::HandId newId;
	//	//		handTracker.startHandTracking(gestures[i].getCurrentPosition(), &newId);
	//	//	}
	//	//}

	//	//const nite::Array<nite::HandData>& hands = handTrackerFrame.getHands();
	//	//for (int i = 0; i < hands.getSize(); ++i)
	//	//{
	//	//	const nite::HandData& hand = hands[i];
	//	//	if (hand.isTracking())
	//	//	{
	//	//		std::cout << "Hand Tracking \n";

	//	//		printf("%d. (%5.2f, %5.2f, %5.2f)\n", hand.getId(), hand.getPosition().x, hand.getPosition().y, hand.getPosition().z);
	//	//	}
	//	//}

	//	if (cloudReadMutex.try_lock())
	//	{
	//		cloudBuffer.swap(rawCloud);
	//		cloudReadMutex.unlock();
	//	}

	//	if (rawCloud) 
	//	{
	//		voxelFilter.setInputCloud(rawCloud);
	//		voxelFilter.setLeafSize(0.02f, 0.02f, 0.02f);
	//		voxelFilter.filter(*filteredCloud);

	//		seg.setInputCloud(filteredCloud);
	//		seg.segment(*inlierIndices, *coefficients);

	//		pointsExtractor.setInputCloud(filteredCloud);
	//		pointsExtractor.setIndices(inlierIndices);
	//		pointsExtractor.setNegative(false);
	//		pointsExtractor.filter(*segmentedCloud);
	//		pointsExtractor.setNegative(true);
	//		pointsExtractor.filter(*wallRemovedCloud);

	//		////Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//		////transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitX()));
	//		////PointCloud<PointXYZ>::Ptr rotatedCloud(new PointCloud<PointXYZ>);
	//		////pcl::transformPointCloud(*wallRemovedCloud, *rotatedCloud, transform);

	//		if (!pclVisualizer.updatePointCloud(wallRemovedCloud, wallRemovedCloudColor, "NoWallCloud"))
	//		{
	//			pclVisualizer.addPointCloud(wallRemovedCloud, wallRemovedCloudColor, "NoWallCloud");
	//		}
	//	
	//		//if (!pclVisualizer.updatePointCloud(segmentedCloud, segmentedCloudColor, "Segmented"))
	//		//{
	//		//	pclVisualizer.addPointCloud(segmentedCloud, segmentedCloudColor, "Segmented");
	//		//}
	//	}

		Update();
	}

	//libusb_exit(ctx);


	///*while (!shouldExit)
	//{
	//	Update();
	//	Draw();

	//	if (GetKeyState(VK_ESCAPE) & 0x8000)
	//	{
	//		shouldExit = true;
	//	}
	//}*/

	Exit();

	return 0;
}
