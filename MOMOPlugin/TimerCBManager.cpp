#include "TimerCBManager.h"
#include "MOMOManager.h"

using namespace MOMO;

boost::shared_ptr<TimerCBManager> TimerCBManager::instance = nullptr;

void TimerCBManager::UpdateCallbacks()
{
	callbacktype::iterator itr = callbackRegister.begin();

	for (; itr != callbackRegister.end(); itr++)
	{
		if (!itr->second->isPaused())
		{
			itr->second->Update(MOMOManager::getInstance()->getDT());
		}
	}
}

void TimerCBManager::RemoveCallback(std::string cbname)
{
	callbackRegister.erase(cbname);
}

void TimerCBManager::AddCallback(std::string cbName, voidcallback callbackfn, int callRate)
{
	callbackRegister.insert(std::make_pair(cbName, boost::make_shared<callbackinfo>(callbackfn, callRate)));
}

boost::shared_ptr<TimerCBManager> TimerCBManager::getInstance()
{
	if (instance == nullptr)
	{
		instance = boost::make_shared<TimerCBManager>();
	}

	return instance;
}

MOMO::TimerCBManager::TimerCBManager()
{
}

MOMO::TimerCBManager::~TimerCBManager()
{

}

callbackinfo::callbackinfo(voidcallback callbackfn, double callRate)
{
	this->callbackfn = callbackfn;
	this->callRate = callRate;

	calculatedDT = (1.0 / callRate);

	timer = 0.0;

	Unpause();
}

void callbackinfo::TimerComplete()
{
	if (!paused)
	{
		if (callbackfn)
		{
			callbackfn();
		}

		ResetTimer();
	}
}

void callbackinfo::ResetTimer()
{
	timer = 0.0;
}

void callbackinfo::Update(double dt)
{
	timer += dt;

	if (timer > calculatedDT)
	{
		TimerComplete();
	}
}
