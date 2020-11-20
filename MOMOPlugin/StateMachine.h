#pragma once

#include <boost/container/map.hpp>
#include <boost/shared_ptr.hpp>
#include "State.h"

using namespace boost;
using namespace std;

namespace MOMO
{
	enum eMOMOStates
	{
		STATE_NONE,
		STATE_STABILITY,
		STATE_WALL_IDENTIFICATION,
		STATE_USER_IDENTIFICATION,
		STATE_USER_TRACKING
	};

	class StateMachine
	{
	public:
		StateMachine();
		~StateMachine();

		void ChangeState(eMOMOStates momoState);
		void EnterCurrentState();
		void UpdateCurrentState();
		void ExitCurrentState();

		eMOMOStates getCurrentState();

		boost::shared_ptr<State> getCurrentStateObject();

		void AddNewState(eMOMOStates targetState, boost::shared_ptr<State> stateObject);
		boost::shared_ptr<State> getStateObject(eMOMOStates state);

		void Init();
		void Deinit();
		void Update();

	private:
		typedef boost::container::map<eMOMOStates, boost::shared_ptr<State>> statesmap;

		statesmap statesMap;

		eMOMOStates eCurrentMOMOState;
	};
}
