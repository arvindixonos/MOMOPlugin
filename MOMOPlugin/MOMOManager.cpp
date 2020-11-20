#include "MOMOManager.h"

#include <boost/chrono.hpp>
#include <boost/make_shared.hpp>


#include "TimerCBManager.h"
#include "State_Stability.h"
#include "State_Wall_Identification.h"

using namespace MOMO;
using namespace pcl;
using namespace openni;

boost::shared_ptr<MOMOManager> MOMOManager::instance = nullptr;

boost::shared_ptr<MOMOManager> MOMOManager::getInstance()
{
	if (instance == nullptr)
	{
		instance = boost::make_shared<MOMOManager>();
	}

	return instance;
}

MOMOManager::MOMOManager()
{
	
}

MOMOManager::~MOMOManager()
{
}

void MOMOManager::Init()
{
	InitVisualizer();
	Initfreenect();
	
	InitOpenNIGrabber();

	InitStateMachine();
	InitServices();

	AddDeviceStates();

	momoStateMachine->ChangeState(eMOMOStates::STATE_STABILITY);
}

void MOMOManager::Initfreenect()
{
	// freenect 
	if (freenect_init(&fnc, NULL) < 0)
	{
		printf("freenect_init() failed\n");
	}

	freenect_select_subdevices(fnc, (freenect_device_flags)(FREENECT_DEVICE_MOTOR));

	int nr_devices = freenect_num_devices(fnc);
	printf("Number of devices found: %d\n", nr_devices);

	if (freenect_open_device(fnc, &fnv, 0) < 0)
	{
		printf("Could not open device\n");
	}
}

void MOMOManager::InitVisualizer()
{
	// Visualizer
	pclVisualizer = boost::make_shared<visualization::PCLVisualizer>("MOMO Preview Window");
	pclVisualizer->setBackgroundColor(0.02, 0.02, 0.02);
	pclVisualizer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1);
}

void MOMOManager::InitStateMachine()
{
	momoStateMachine = boost::make_shared<StateMachine>();
	momoStateMachine->Init();
	momoStateMachine->AddNewState(eMOMOStates::STATE_STABILITY, boost::make_shared<State_Stability>());
	momoStateMachine->ChangeState(STATE_STABILITY);
}

void MOMOManager::InitServices()
{
	stabilityCheckService = boost::make_shared<StabilityCheckService>();
	stabilityCheckService->Init(boost::bind(&MOMOManager::OnDeviceStable, this), boost::bind(&MOMOManager::OnDeviceUnstable, this));
}

void MOMOManager::AddDeviceStates()
{
	momoStateMachine->AddNewState(eMOMOStates::STATE_STABILITY, boost::make_shared<State_Stability>());
	momoStateMachine->AddNewState(eMOMOStates::STATE_WALL_IDENTIFICATION, boost::make_shared<State_Wall_Identification>());
}

void MOMOManager::InitOpenNIGrabber()
{
	openni::Status opennistatus = OpenNI::initialize();
	openniDevice = boost::make_shared<openni::Device>();

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return;
	}

	opennistatus = openniDevice->open(openni::ANY_DEVICE);

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Unable to open device");
		return;
	}

	const std::string device_id = openniDevice->getDeviceInfo().getUri();
	openni2Grabber = boost::make_shared<pcl::io::OpenNI2Grabber>(device_id);
	boost::function<void(const PointCloud<PointXYZ>::ConstPtr&)> f = boost::bind(&MOMOManager::CloudAvailable, this);

	//openni2Grabber->registerCallback(f);
	//openni2Grabber->start();
}


void MOMO::MOMOManager::CloudAvailable(const PointCloud<PointXYZ>::ConstPtr &cloud)
{
	/*boost::mutex::scoped_lock lock(cloudReadMutex);
	cloudBuffer = cloud->makeShared();

	framecounter++;*/
	std::cout << "Got Frame: "<<"\n";
}


void MOMO::MOMOManager::Update(double dt)
{
	currentDT = dt;

	momoStateMachine->Update();

	TimerCBManager::getInstance()->UpdateCallbacks();

	pclVisualizer->spinOnce();
}

void MOMOManager::DeviceUnstable()
{
	deviceStable = false;

	momoStateMachine->ChangeState(eMOMOStates::STATE_STABILITY);
}

void MOMOManager::DeviceStable()
{
	deviceStable = true;

	momoStateMachine->ChangeState(eMOMOStates::STATE_WALL_IDENTIFICATION);
}

void MOMOManager::UpdateTiltState()
{
	freenect_update_tilt_state(fnv);
}

void MOMOManager::OnDeviceStable()
{
	DeviceStable();
}

void MOMOManager::OnDeviceUnstable()
{
	DeviceUnstable();
}

boost::array<double, 3> MOMOManager::GetAccelerometerValues()
{
	boost::array<double, 3> ret;
	double x, y, z;
	
	freenect_raw_tilt_state* tiltState = freenect_get_tilt_state(fnv);
	freenect_get_mks_accel(tiltState, &x, &y, &z);

	ret[0] = x;
	ret[1] = y;
	ret[2] = z;

	return ret;
}

bool MOMOManager::isStopped()
{
	return pclVisualizer->wasStopped();
}

void MOMOManager::Exit()
{
	// freenect

	freenect_close_device(fnv);
	freenect_shutdown(fnc);

	stabilityCheckService->Deinit();

	fnc = nullptr;
	fnv = nullptr;
}
