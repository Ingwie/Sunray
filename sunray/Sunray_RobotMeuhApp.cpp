/***************************************************************
 * Name:      Sunray_RobotMeuhApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Bracame ()
 * Created:   2024-10-01
 * Copyright: Bracame ()
 * License:
 **************************************************************/

#include "Sunray_RobotMeuhApp.h"

//(*AppHeaders
#include "Sunray_RobotMeuhMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(Sunray_RobotMeuhApp);

bool Sunray_RobotMeuhApp::OnInit()
{
  //(*AppInitialize
  bool wxsOK = true;
  wxInitAllImageHandlers();
  if ( wxsOK )
    {
      Sunray_RobotMeuhFrame* Frame = new Sunray_RobotMeuhFrame(0);
      Frame->Show();
      SetTopWindow(Frame);
    }
  //*)
  return wxsOK;

}
