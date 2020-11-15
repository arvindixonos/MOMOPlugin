#include <OpenNI.h>
#include <Nite.h>

using namespace openni;
using namespace nite;


int main()
{
	openni::Status rc = OpenNI::initialize();
	nite::Status nc = NiTE::initialize();

	if (rc != openni::Status::STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != openni::Status::STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	HandTracker* handTracker = new HandTracker();

	nc = handTracker->create(&device);

	handTracker->startGestureDetection(GestureType::GESTURE_WAVE);

	device.close();
	
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	delete handTracker;

	return 0;
}