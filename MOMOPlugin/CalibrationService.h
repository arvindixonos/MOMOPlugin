#pragma once

#include "Service.h"
#include <boost/shared_ptr.hpp>
#include <pcl/visualization/pcl_painter2D.h>
#include <vtkRenderer.h>
#include <vtkWindow.h>

using namespace pcl;

namespace MOMO
{
	class CalibrationService : public Service
	{
	public:
		CalibrationService();
		~CalibrationService();

		void InitStartService();
		void UpdateService();
		void StopService();

	private:
		boost::shared_ptr <visualization::PCLPainter2D> uiPainter;
		vtkWindow* uiWindow;
		vtkRenderer* uiRenderer;

		boost::shared_ptr <visualization::PCLPainter2D> projectorPainter;
		vtkWindow* projectorWindow;
		vtkRenderer* projectorRenderer;
	};
}