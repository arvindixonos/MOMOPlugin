#pragma once

#include <vtkInteractorStyleUser.h>

namespace MOMO
{
	class InteractionHandler : public vtkInteractorStyleUser
	{
	public:
		InteractionHandler();
		~InteractionHandler();

		void OnLeftButtonDown();

		void GetMousePosition(int& x, int& y);

	private:

	};
}