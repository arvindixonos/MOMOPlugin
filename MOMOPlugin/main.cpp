#include <OpenNI.h>
#include <Nite.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_device_manager.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace openni;
using namespace nite;
//using namespace pcl;
using namespace pcl::io::openni2;

std::shared_ptr<openni::Device> openniDevice;
std::shared_ptr<pcl::io::openni2::OpenNI2Device> pclDevice;
std::shared_ptr<pcl::io::OpenNI2Grabber> openni2Grabber;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
boost::mutex mutex;
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr buffer;
HandTracker handTracker;
bool shouldExit = false;
int framecounter = 0;

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	/*viewer->addPointCloud(cloud);*/
	
	//seg.setInputCloud(cloud);
	//seg.segment(*inliers, *coefficients);

	//static unsigned count = 0;
	//static double last = pcl::getTime();
	//if (++count == 30)
	//{
	//	double now = pcl::getTime();
	//	std::cout << "distance of center pixel :" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count) / double(now - last) << " Hz" << std::endl;
	//	count = 0;
	//	last = now;
	//}

	boost::mutex::scoped_lock lock(mutex);
	buffer = cloud->makeShared();

	//if (!viewer->updatePointCloud(cloud, "cloud"))
	//{
	//	viewer->addPointCloud(cloud, "cloud");
	//}

	framecounter++;
	std::cout << "Got Frame: "<<framecounter<<"\n";

	//if (!viewer->updatePointCloud(cloud, "Cloud")) {
	//	viewer->addPointCloud(cloud, "Cloud");
	//}
}

int Setup()
{
	openni::Status opennistatus = OpenNI::initialize();

	openniDevice = std::make_shared<openni::Device>();

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	opennistatus = openniDevice->open(ANY_DEVICE);
	//openniDevice->setImageRegistrationMode(ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Unable to open device");
		return 1;
	}	

	const std::string device_id = openniDevice->getDeviceInfo().getUri();

	//pcl::io::openni2::OpenNI2Device::Ptr testDevice = OpenNI2DeviceManager::getInstance()->getDevice(device_id);

	//nite::Status nitestatus = NiTE::initialize();

	////nitestatus = handTracker.create(&openniDevice);
	//
	//handTracker.startGestureDetection(GestureType::GESTURE_WAVE);

	//pclDevice = std::make_shared<pcl::io::openni2::OpenNI2Device>(openniDevice);

	openni2Grabber = std::make_shared<pcl::io::OpenNI2Grabber>(device_id);

	// make callback function from member function
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(cloud_cb_, _1);
	
	//seg.setOptimizeCoefficients(true);
	//// Mandatory
	//seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setDistanceThreshold(0.01);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::io::loadPCDFile("test.pcd", *source_cloud);

	viewer->setBackgroundColor(0.7, 0.05, 0.05, 0);
	//viewer->addPointCloud(source_cloud);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
	viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();

	openni2Grabber->registerCallback(f);
	openni2Grabber->start();

 	return 0;
}

void Update()
{

}

void Draw()
{

}

void Exit()
{
	//handTracker.stopGestureDetection(GestureType::GESTURE_WAVE);

	//handTracker.destroy();

	openni2Grabber->stop();

	openniDevice->close();

	//nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

int main()
{
	Setup();

	while (!viewer->wasStopped()) 
	{   // Display the visualizer until the 'q' key is pressed
		viewer->spinOnce();

		std::cout << "Hello" << "\n";

		//boost::this_thread::sleep(boost::posix_time::seconds(1));

		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

		if (mutex.try_lock()) {
			buffer.swap(cloud);
			mutex.unlock();
		}

		if (cloud) {
			if (!viewer->updatePointCloud(cloud, "Cloud")) {
				viewer->addPointCloud(cloud, "Cloud");
			}
		}
	}

	/*while (!shouldExit)
	{
		Update();
		Draw();

		if (GetKeyState(VK_ESCAPE) & 0x8000)
		{
			shouldExit = true;
		}
	}*/

	Exit();

	return 0;
}