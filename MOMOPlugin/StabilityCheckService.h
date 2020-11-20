#pragma once
#include <boost/array.hpp>
#include "Constants.h"

namespace MOMO
{
	class StabilityCheckService
	{
	public:
		StabilityCheckService();
		~StabilityCheckService();
		
		void Init(voidcallback deviceStable_cb, voidcallback  deviceUnstable_cb);
		void Deinit();

		void ResetAll();
		void ResetRateCounter();
		void ResetAccumulators();

		void Update();

	private:
		boost::array<double, 3> previousAccValues;

		voidcallback deviceStableCallback;
		voidcallback deviceUnstableCallback;
		
		int rateCounter;
		double accDeviationX;
		double accDeviationY;
		double accDeviationZ;
	};
}