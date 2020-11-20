#pragma once

#include <boost/container/map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>

#include "Constants.h"

//using namespace ;

namespace MOMO
{
	struct callbackinfo
	{
	private:
		voidcallback callbackfn;
		double callRate;
		double calculatedDT;

		double timer;
		
		bool paused;
	public:

		callbackinfo(voidcallback callbackfn, double callRate);

		void TimerComplete();

		void ResetTimer();

		void Update(double dt);

		bool isPaused() const { return paused; }
		void Pause() 
		{ 
			paused = true; 
		}
		
		void Unpause() 
		{ 
			paused = false; 
		}
	};

	class TimerCBManager : private boost::noncopyable
	{
	public:
		static boost::shared_ptr<TimerCBManager> getInstance();

		void AddCallback(std::string cbName, voidcallback callbackfn, int callRate);
		
		void RemoveCallback(std::string cbname);

		void UpdateCallbacks();

		TimerCBManager();
		~TimerCBManager();

	private:
		static boost::shared_ptr<TimerCBManager> instance;

		typedef boost::container::map<std::string, boost::shared_ptr<callbackinfo>> callbacktype;
		callbacktype callbackRegister;
	};
}