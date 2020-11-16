#include <OpenNI.h>
#include <Nite.h>


using namespace openni;
using namespace nite;


Device device;
HandTracker handTracker;
bool shouldExit = false;


int Setup()
{
	openni::Status opennistatus = OpenNI::initialize();

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	opennistatus = device.open(ANY_DEVICE);

	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Unable to open device");
		return 1;
	}

	VideoStream depth;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		opennistatus = depth.create(device, SENSOR_DEPTH);
		if (opennistatus != openni::Status::STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}

	opennistatus = depth.start();
	if (opennistatus != openni::Status::STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	
	nite::Status nitestatus = NiTE::initialize();
	
	nitestatus = handTracker.create(&device);

	handTracker.startGestureDetection(GestureType::GESTURE_WAVE);

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
	handTracker.stopGestureDetection(GestureType::GESTURE_WAVE);

	device.close();

	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

int main()
{
	Setup();

	while (!shouldExit)
	{
		Update();
		Draw();

		if (GetKeyState(VK_ESCAPE) & 0x8000)
		{
			shouldExit = true;
		}
	}
	
	Exit();

	return 0;
}