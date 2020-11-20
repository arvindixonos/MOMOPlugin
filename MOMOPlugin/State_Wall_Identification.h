#pragma once

#include "State.h"

namespace MOMO
{
	class State_Wall_Identification : public State
	{
	public:
		State_Wall_Identification();
		~State_Wall_Identification();

		void EnterState();
		void UpdateState();
		void ExitState();

	private:

	};
}