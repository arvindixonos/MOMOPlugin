#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "CalibrationService.h"
#include "Constants.h"


MOMO::CalibrationService::CalibrationService()
{

}

MOMO::CalibrationService::~CalibrationService()
{

}

void MOMO::CalibrationService::InitStartService()
{
	uiPainter = boost::make_shared<visualization::PCLPainter2D>();
	uiRenderer = uiPainter->GetScene()->GetRenderer();
	uiWindow = uiRenderer->GetVTKWindow();
	uiWindow->SetWindowName("UI Window");
	uiWindow->SetPosition(0, 0);
	uiPainter->setWindowSize(CALIBRATION_UI_SIZE_X, CALIBRATION_UI_SIZE_Y);
	uiPainter->GetScene()->GetRenderer()->SetViewport(0.0, 0.0, 1.0, 1.0);
	uiPainter->setBrushColor(255, 0, 0, 255);


	projectorPainter = boost::make_shared<visualization::PCLPainter2D>();
	projectorRenderer = projectorPainter->GetScene()->GetRenderer();
	projectorWindow = projectorRenderer->GetVTKWindow();
	projectorWindow->SetWindowName("Projection Window");
	projectorWindow->SetPosition(1920, 0);
	projectorPainter->setWindowSize(CALIBRATION_PROJECTION_SIZE_X, CALIBRATION_PROJECTION_SIZE_Y);
	projectorPainter->GetScene()->GetRenderer()->SetViewport(0.0, 0.0, 1.0, 1.0);
	projectorPainter->setBrushColor(255, 0, 0, 255);

	started = true;
}

void MOMO::CalibrationService::UpdateService()
{
	uiPainter->spinOnce();
	projectorPainter->spinOnce();
}

void MOMO::CalibrationService::StopService()
{
	uiPainter->Delete();
	projectorPainter->Delete();

	started = false;
}
