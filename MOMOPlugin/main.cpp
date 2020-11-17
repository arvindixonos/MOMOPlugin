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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>


using namespace openni;
using namespace nite;
//using namespace pcl;
using namespace pcl::io::openni2;

std::shared_ptr<openni::Device> openniDevice;
std::shared_ptr<pcl::io::openni2::OpenNI2Device> pclDevice;
std::shared_ptr<pcl::io::OpenNI2Grabber> openni2Grabber;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
boost::mutex mutex;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudBuffer;
pcl::PointCloud<pcl::PointXYZ>::ConstPtr rawCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr wallRemovedCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> segmentedCloudColor(segmentedCloud, 255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> wallRemovedCloudColor(wallRemovedCloud, 255, 255, 255);

pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
pcl::ExtractIndices<pcl::PointXYZ> pointsExtractor;


HandTracker handTracker;
bool shouldExit = false;
int framecounter = 0;

pcl::visualization::PCLVisualizer pclVisualizer("PCL Visualizer");
//pcl::visualization::CloudViewer cloudViewer("Cloud Viewer");

void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	boost::mutex::scoped_lock lock(mutex);
	cloudBuffer = cloud->makeShared();

	framecounter++;
	//std::cout << "Got Frame: "<<framecounter<<"\n";
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

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Unable to open device");
		return 1;
	}	

	const std::string device_id = openniDevice->getDeviceInfo().getUri();
	openni2Grabber = std::make_shared<pcl::io::OpenNI2Grabber>(device_id);

	// make callback function from member function
	boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind(cloud_cb_, _1);
	
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);

	pclVisualizer.setBackgroundColor(0.02, 0.02, 0.02);
	//pclVisualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.5);
	pclVisualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
	//pclVisualizer.addCoordinateSystem(1.0);

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

	//while (!cloudViewer.wasStopped())
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;

	//	if (mutex.try_lock()) 
	//	{
	//		buffer.swap(cloud);
	//		mutex.unlock();
	//	}

	//	if (cloud)
	//	{
	//		//seg.setInputCloud(cloud);

	////		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	////		pcl::PointIndices inlierIndices;
	////		seg.segment(inlierIndices, *coefficients);

	//		cloudViewer.showCloud(cloud);
	//	}
	//}

	while (!pclVisualizer.wasStopped()) 
	{  
		if (mutex.try_lock()) 
		{
			cloudBuffer.swap(rawCloud);
			mutex.unlock();
		}

		if (rawCloud) 
		{
			voxelFilter.setInputCloud(rawCloud);
			voxelFilter.setLeafSize(0.02f, 0.02f, 0.02f);
			voxelFilter.filter(*filteredCloud);

			seg.setInputCloud(filteredCloud);
			seg.segment(*inlierIndices, *coefficients);
			std::cout << inlierIndices->indices.size() << "\n";

			pointsExtractor.setInputCloud(filteredCloud);
			pointsExtractor.setIndices(inlierIndices);
			pointsExtractor.setNegative(false);
			pointsExtractor.filter(*segmentedCloud);
			pointsExtractor.setNegative(true);
			pointsExtractor.filter(*wallRemovedCloud);

			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitX()));

			pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
			/*pcl::transformPointCloud(*wallRemovedCloud, *rotatedCloud, transform);*/

			if (!pclVisualizer.updatePointCloud(wallRemovedCloud, wallRemovedCloudColor, "NoWallCloud"))
			{
				pclVisualizer.addPointCloud(wallRemovedCloud, wallRemovedCloudColor, "NoWallCloud");
			}
		
			if (!pclVisualizer.updatePointCloud(segmentedCloud, segmentedCloudColor, "Segmented"))
			{
				pclVisualizer.addPointCloud(segmentedCloud, segmentedCloudColor, "Segmented");
			}

			//pclVisualizer.resetCameraViewpoint("NoWallCloud");

			pclVisualizer.spinOnce();

			//boost::this_thread::sleep(boost::posix_time::milliseconds(5));
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