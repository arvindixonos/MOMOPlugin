#pragma once
#include <boost/array.hpp>

#include "Service.h"
#include "Constants.h"

namespace MOMO
{
	class StabilityCheckService : public Service
	{
	public:
		StabilityCheckService();
		~StabilityCheckService();
		
		void InitStartService();
		void UpdateService();
		void StopService();

		void Init(voidcallback deviceStable_cb, voidcallback  deviceUnstable_cb);
		void Deinit();

		void ResetAllCounters();
		void ResetRateCounter();
		void ResetAccumulators();

		bool isPaused() const { return pause; }
		void Pause() { pause = true; }
		void Unpause() { pause = false; }

	private:
		boost::array<double, 3> previousAccValues;

		voidcallback deviceStableCallback;
		voidcallback deviceUnstableCallback;
		
		int rateCounter;
		double accDeviationX;
		double accDeviationY;
		double accDeviationZ;

		bool pause;
	};
}