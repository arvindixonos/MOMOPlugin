#pragma once

#include <stdio.h>

namespace MOMO
{
	class State
	{
	public:
		State() : paused(false)
		{
		}

		~State() {}

		virtual void EnterState() = 0;
		virtual void UpdateState() = 0;
		virtual void ExitState() = 0;

		bool isPaused() const { return paused; }
		void Pause() { paused = true; }
		void Unpause() { paused = false; }

	protected:
		bool paused;


	};

	
}