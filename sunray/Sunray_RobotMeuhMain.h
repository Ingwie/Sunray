/***************************************************************
 * Name:      Sunray_RobotMeuhMain.h
 * Purpose:   Defines Application Frame
 * Author:    Bracame ()
 * Created:   2024-10-01
 * Copyright: Bracame ()
 * License:
 **************************************************************/

#ifndef SUNRAY_ROBOTMEUHMAIN_H
#define SUNRAY_ROBOTMEUHMAIN_H

//(*Headers(Sunray_RobotMeuhFrame)
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
//*)
#include <wx/tglbtn.h>

#include <wx/log.h>

#include "src/robot.h"


extern void start();
class StartThread: public wxThread
{
public:
  StartThread() : wxThread(wxTHREAD_DETACHED)
  {
    if(wxTHREAD_NO_ERROR == Create())
      {
        Run();
      }
  }
protected:
  virtual void* Entry()
  {
    start();

    while (TestDestroy())
      {
        Delete();
      }
    return 0;
  }
};


extern void loop();
class LoopThread: public wxThread
{
public:
  LoopThread() : wxThread(wxTHREAD_DETACHED)
  {
    if(wxTHREAD_NO_ERROR == Create())
      {
        Run();
      }
  }
protected:
  virtual void* Entry()
  {

    loop();

    while (TestDestroy())
      {
        Delete();
      }
    return 0;
  }
};


class Sunray_RobotMeuhFrame: public wxFrame
{
public:

  Sunray_RobotMeuhFrame(wxWindow* parent,wxWindowID id = -1);
  virtual ~Sunray_RobotMeuhFrame();

  StartThread * startThread;
  LoopThread * loopThread;

private:

  //(*Handlers(Sunray_RobotMeuhFrame)
  void OnQuit(wxCommandEvent& event);
  void OnAbout(wxCommandEvent& event);
  void OnButton1Click(wxCommandEvent& event);
  void OnButton2Click(wxCommandEvent& event);
  void OnONOFFButtonClick(wxCommandEvent& event);
  void OnTimer1mSTrigger(wxTimerEvent& event);
  //*)

  //(*Identifiers(Sunray_RobotMeuhFrame)
  static const long ID_BUTTON1;
  static const long ID_BUTTON2;
  static const long ID_BUTTON3;
  static const long ID_TEXTCTRL1;
  static const long ID_PANEL1;
  static const long ID_MENUITEM1;
  static const long idMenuAbout;
  static const long ID_STATUSBAR1;
  static const long ID_TIMER1;
  //*)

  //(*Declarations(Sunray_RobotMeuhFrame)
  wxButton* Button1;
  wxButton* Button2;
  wxPanel* Panel1;
  wxStatusBar* StatusBar1;
  wxTextCtrl* TextCtrl1;
  wxTimer Timer1mS;
  wxToggleButton* ONOFFButton;
  //*)

  wxLogTextCtrl* logWiew;

  DECLARE_EVENT_TABLE()
};

#endif // SUNRAY_ROBOTMEUHMAIN_H
