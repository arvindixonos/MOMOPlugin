
#include "StabilityCheckService.h"
#include "TimerCBManager.h"
#include "MOMOManager.h"
#include "Constants.h"


MOMO::StabilityCheckService::StabilityCheckService()
{
}

MOMO::StabilityCheckService::~StabilityCheckService()
{

}

void MOMO::StabilityCheckService::InitStartService()
{
	started = true;

	ResetAllCounters();
}

void MOMO::StabilityCheckService::StopService()
{
	Deinit();

	started = false;
}

void MOMO::StabilityCheckService::Init(voidcallback deviceStable_cb, voidcallback  deviceUnstable_cb)
{
	deviceStableCallback = deviceStable_cb;
	deviceUnstableCallback = deviceUnstable_cb;

	TimerCBManager::getInstance()->AddCallback("Stability Check Update", boost::bind(&StabilityCheckService::UpdateService, this), STABLITY_SERVICE_UPDATE_RATE);
}

void MOMO::StabilityCheckService::UpdateService()
{
	if (pause)
	{
		return;
	}

	MOMOManager::getInstance()->UpdateTiltState();

	boost::array<double, 3> accValues = MOMOManager::getInstance()->GetAccelerometerValues();

	accDeviationX += std::abs(accValues[X_VAL] - previousAccValues[X_VAL]);
	accDeviationY += std::abs(accValues[Y_VAL] - previousAccValues[Y_VAL]);
	accDeviationZ += std::abs(accValues[Z_VAL] - previousAccValues[Z_VAL]);

	rateCounter += 1;

	//printf("RATE: %.3f %.3f %.3f\n", accDeviationX, accDeviationY, accDeviationZ);

	previousAccValues = accValues;

	if (MOMOManager::getInstance()->isDeviceStable())
	{
		if (rateCounter >= STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES)
		{
			double meanX = accDeviationX / (double)STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES;
			double meanY = accDeviationY / (double)STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES;
			double meanZ = accDeviationZ / (double)STABILITY_DEVICE_UNSTABLE_CHECK_FRAMES;

			//printf("STABLE DEVICE: %.3f %.3f %.3f \n", meanX, meanY, meanZ);

			if (meanX > STABILITY_DEVIATION_THRESHOLD ||
				meanY > STABILITY_DEVIATION_THRESHOLD ||
				meanZ > STABILITY_DEVIATION_THRESHOLD)
			{
				if (deviceUnstableCallback)
				{
					printf("DEVICE UNSTABLE: %.3f %.3f %.3f \n", meanX, meanY, meanZ);

					deviceUnstableCallback();
				}
			}

			ResetAllCounters();
		}	
	}
	else
	{
		if (rateCounter >= STABILITY_DEVICE_STABLE_CHECK_FRAMES)
		{
			double meanX = accDeviationX / (double)STABILITY_DEVICE_STABLE_CHECK_FRAMES;
			double meanY = accDeviationY / (double)STABILITY_DEVICE_STABLE_CHECK_FRAMES;
			double meanZ = accDeviationZ / (double)STABILITY_DEVICE_STABLE_CHECK_FRAMES;

			if (meanX < STABILITY_DEVIATION_THRESHOLD &&
				meanY < STABILITY_DEVIATION_THRESHOLD &&
				meanZ < STABILITY_DEVIATION_THRESHOLD)
			{
				if (deviceStableCallback)
				{
					printf("DEVICE STABLE: %.3f %.3f %.3f \n", meanX, meanY, meanZ);

					deviceStableCallback();
				}
			}

			ResetAllCounters();
		}
	}
}

void MOMO::StabilityCheckService::ResetAllCounters()
{
	ResetAccumulators();
	ResetRateCounter();
}

void MOMO::StabilityCheckService::Deinit()
{
	ResetAllCounters();

	TimerCBManager::getInstance()->RemoveCallback("Stability Check Update");
}

void MOMO::StabilityCheckService::ResetRateCounter()
{
	rateCounter = 0;
}

void MOMO::StabilityCheckService::ResetAccumulators()
{
	accDeviationX = accDeviationY = accDeviationZ = 0.0;
}