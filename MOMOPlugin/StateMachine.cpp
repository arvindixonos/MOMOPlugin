#include <boost/make_shared.hpp>
#include "StateMachine.h"

using namespace MOMO;

StateMachine::StateMachine()
	: eCurrentMOMOState(eMOMOStates::STATE_NONE)
{
}

StateMachine::~StateMachine()
{
	Deinit();
}

void StateMachine::ChangeState(eMOMOStates momoState)
{
	if (eCurrentMOMOState == momoState)
	{
		return;
	}

	ExitCurrentState();

	eCurrentMOMOState = momoState;
	
	EnterCurrentState();
}

MOMO::eMOMOStates StateMachine::getCurrentState()
{
	return eCurrentMOMOState;
}

void StateMachine::AddNewState(eMOMOStates targetState, boost::shared_ptr<State> stateObject)
{
	statesMap.insert(std::make_pair(targetState, stateObject));
}

void StateMachine::Init()
{
}

boost::shared_ptr<MOMO::State> StateMachine::getCurrentStateObject()
{
	if (eCurrentMOMOState == STATE_NONE)
		return nullptr;

	boost::shared_ptr<MOMO::State> currentState = statesMap[eCurrentMOMOState];

	return currentState;
}

void StateMachine::EnterCurrentState()
{
	boost::shared_ptr<MOMO::State> currentState = getCurrentStateObject();

	if (currentState)
	{
		currentState->EnterState();
	}
}

void StateMachine::UpdateCurrentState()
{
	boost::shared_ptr<MOMO::State> currentState = getCurrentStateObject();

	if (currentState)
	{
		if (!currentState->isPaused())
		{
			currentState->UpdateState();
		}
	}
}

void StateMachine::ExitCurrentState()
{
	boost::shared_ptr<MOMO::State> currentState = getCurrentStateObject();

	if (currentState)
	{
		currentState->ExitState();
	}
}

void StateMachine::Deinit()
{
	ExitCurrentState();

	eCurrentMOMOState = eMOMOStates::STATE_NONE;
}

boost::shared_ptr<MOMO::State> StateMachine::getStateObject(eMOMOStates state)
{
	if (statesMap.find(state) != statesMap.end())
	{
		return statesMap[state];
	}

	return nullptr;
}

void MOMO::StateMachine::Update()
{
	UpdateCurrentState();
}
