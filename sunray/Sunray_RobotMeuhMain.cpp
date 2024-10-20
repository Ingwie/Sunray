/***************************************************************
 * Name:      Sunray_RobotMeuhMain.cpp
 * Purpose:   Code for Application Frame
 * Author:    Bracame ()
 * Created:   2024-10-01
 * Copyright: Bracame ()
 * License:
 **************************************************************/

#include "Sunray_RobotMeuhMain.h"
#include <wx/msgdlg.h>
#include <wx/filename.h>
#include <wx/stdpaths.h>

//(*InternalHeaders(Sunray_RobotMeuhFrame)
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

wxString AppPath;

//helper functions
enum wxbuildinfoformat
{
  short_f, long_f
};

wxString wxbuildinfo(wxbuildinfoformat format)
{
  wxString wxbuild(wxVERSION_STRING);

  if (format == long_f )
    {
#if defined(__WXMSW__)
      wxbuild << _T("-Windows");
#elif defined(__UNIX__)
      wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
      wxbuild << _T("-Unicode build");
#else
      wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

  return wxbuild;
}

//(*IdInit(Sunray_RobotMeuhFrame)
const long Sunray_RobotMeuhFrame::ID_BUTTON1 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_BUTTON2 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_BUTTON3 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_TEXTCTRL1 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_PANEL1 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_MENUITEM1 = wxNewId();
const long Sunray_RobotMeuhFrame::idMenuAbout = wxNewId();
const long Sunray_RobotMeuhFrame::ID_STATUSBAR1 = wxNewId();
const long Sunray_RobotMeuhFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(Sunray_RobotMeuhFrame,wxFrame)
  //(*EventTable(Sunray_RobotMeuhFrame)
  //*)
END_EVENT_TABLE()

Sunray_RobotMeuhFrame::Sunray_RobotMeuhFrame(wxWindow* parent,wxWindowID id)
{
  //(*Initialize(Sunray_RobotMeuhFrame)
  wxMenu* Menu1;
  wxMenu* Menu2;
  wxMenuBar* MenuBar1;
  wxMenuItem* MenuItem1;
  wxMenuItem* MenuItem2;

  Create(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
  SetClientSize(wxSize(1057,452));
  Panel1 = new wxPanel(this, ID_PANEL1, wxPoint(352,104), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
  Button1 = new wxButton(Panel1, ID_BUTTON1, _("init"), wxPoint(24,24), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
  Button2 = new wxButton(Panel1, ID_BUTTON2, _("loop"), wxPoint(32,104), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
  ONOFFButton = new wxToggleButton(Panel1, ID_BUTTON3, _("ON"), wxPoint(40,192), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
  TextCtrl1 = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxPoint(152,24), wxSize(864,376), wxTE_MULTILINE|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
  TextCtrl1->SetForegroundColour(wxColour(5,255,21));
  TextCtrl1->SetBackgroundColour(wxColour(0,0,0));
  wxFont TextCtrl1Font(10,wxFONTFAMILY_DEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,_T("System-ui"),wxFONTENCODING_DEFAULT);
  TextCtrl1->SetFont(TextCtrl1Font);
  MenuBar1 = new wxMenuBar();
  Menu1 = new wxMenu();
  MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
  Menu1->Append(MenuItem1);
  MenuBar1->Append(Menu1, _("&File"));
  Menu2 = new wxMenu();
  MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
  Menu2->Append(MenuItem2);
  MenuBar1->Append(Menu2, _("Help"));
  SetMenuBar(MenuBar1);
  StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
  int __wxStatusBarWidths_1[1] = { -1 };
  int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
  StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
  StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
  SetStatusBar(StatusBar1);
  Timer1mS.SetOwner(this, ID_TIMER1);
  Timer1mS.Start(5, false);

  Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnButton1Click);
  Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnButton2Click);
  Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnONOFFButtonClick);
  Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnQuit);
  Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnAbout);
  Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&Sunray_RobotMeuhFrame::OnTimer1mSTrigger);
  //*)

  wxFileName appPathWithExeName = wxStandardPaths::Get().GetExecutablePath();
  AppPath = appPathWithExeName.GetPath();


  logWiew = new wxLogTextCtrl(TextCtrl1);
  logWiew->SetActiveTarget(logWiew);
  //logWiew->SetThreadActiveTarget(logWiew);
  logWiew->DisableTimestamp();
  //logWiew->Suspend();

  //ONOFFButton->SetValue(true);

}

Sunray_RobotMeuhFrame::~Sunray_RobotMeuhFrame()
{
  //(*Destroy(Sunray_RobotMeuhFrame)
  //*)
}

void Sunray_RobotMeuhFrame::OnQuit(wxCommandEvent& event)
{
  Close();
}

void Sunray_RobotMeuhFrame::OnAbout(wxCommandEvent& event)
{
  wxString msg = wxbuildinfo(long_f);
  wxMessageBox(msg, _("Welcome to..."));
}

void Sunray_RobotMeuhFrame::OnButton1Click(wxCommandEvent& event)
{
  //start();
  startThread = new(StartThread);
}

void Sunray_RobotMeuhFrame::OnButton2Click(wxCommandEvent& event)
{
  loopThread = new(LoopThread);
}

void Sunray_RobotMeuhFrame::OnONOFFButtonClick(wxCommandEvent& event)
{
  /*event.Skip();
  if
    {
      //ON

    }
  else
    {

    }*/
}

void Sunray_RobotMeuhFrame::OnTimer1mSTrigger(wxTimerEvent& event)
{
  event.Skip();

  if (ONOFFButton->GetValue())
    {
      if (StartRunDone == false)
        {
          if (StartIsRunning == false)
            {
              startThread = new(StartThread);
            }
        }
      else if (LoopIsRunning == false)
        {
          loopThread = new(LoopThread);
        }
    }
}
