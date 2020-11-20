#pragma once

#include "State.h"

namespace MOMO
{
	class State_Stability : public State
	{
	public:
		State_Stability();
		~State_Stability();

		void EnterState();
		void UpdateState();
		void ExitState();

	private:

	};
}