#pragma once

#include <stdio.h>

namespace MOMO
{
	class Service
	{
	public:
		Service() : paused(false), started(false)
		{

		}

		~Service() {}

		virtual void InitStartService() = 0;
		virtual void UpdateService() = 0;
		virtual void StopService() = 0;

		bool isPaused() const { return paused; }
		void Pause() { paused = true; }
		void Unpause() { paused = false; }

		bool isStarted() const { return started; }
		bool isRunning() const 
		{ 
			return started; 
		}

	protected:
		bool started;
		bool paused;
	};
}