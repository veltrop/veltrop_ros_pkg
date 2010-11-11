#include <wx/wx.h>
#include <ros/ros.h>
#include <roboard_servos/SetGyroCompensation.h>
#include "gyro_compensation.h"

namespace roboard_servos
{

ros::NodeHandlePtr n_;
GyroCompensatonList gyro_pitch_compensation_;
GyroCompensatonList gyro_roll_compensation_;

enum
{
  ID_Quit = 1,
  ID_OpenFile,
  ID_SaveFile,
  ID_About,
  ID_Slider,
  ID_Text
};

struct GyroRow
{
  wxStaticText* label_;
  wxSlider *pwm_slider_;
  wxTextCtrl *pwm_text_;
};
typedef std::map<GyroCompensaton*, GyroRow> GyroRowLibrary;

class GyroPanel : public wxPanel
{
public:
  GyroPanel(wxFrame *parent)
           : wxPanel(parent, wxID_ANY)
  {      
    // Load Gyro Compensation configuration
    gyro_pitch_compensation_.loadFromParamServer("gyro_pitch_conf");
    gyro_roll_compensation_.loadFromParamServer("gyro_roll_conf");
  
    wxFlexGridSizer *sizer = new wxFlexGridSizer(3);
    sizer->AddGrowableCol(0);
    sizer->AddGrowableCol(1);  
      
    AddGyroRows(gyro_pitch_compensation_, sizer);
    AddGyroRows(gyro_roll_compensation_, sizer);
    
    sizer->SetSizeHints(this);
    SetSizer(sizer);
                          
    Connect(ID_Slider, wxEVT_COMMAND_SLIDER_UPDATED, 
            wxScrollEventHandler(GyroPanel::OnScroll));                       
    Connect(ID_Text, wxEVT_COMMAND_TEXT_ENTER,
            wxCommandEventHandler(GyroPanel::OnEnter)); 
           
    gyro_pitch_pub_ = n_->advertise<roboard_servos::SetGyroCompensation>
                          ("/set_pitch_compensation", 20);
    gyro_roll_pub_ = n_->advertise<roboard_servos::SetGyroCompensation>
                         ("/set_roll_compensation", 20);            
  }

  void OnScroll(wxScrollEvent& event)
  {
    GyroRowLibrary::iterator i = gyro_rows_.begin();
    for(; i != gyro_rows_.end(); ++i)
    {
      GyroCompensaton* gyro = i->first;
      GyroRow& row = i->second;    
      int val = row.pwm_slider_->GetValue();
      row.pwm_text_->SetValue(wxString::Format(_("%d"), val));
      gyro->modifier10_ = val;
    }

    SendGyroUpdate();
  }
  
  void OnEnter(wxCommandEvent& event)
  {
    GyroRowLibrary::iterator i = gyro_rows_.begin();
    for(; i != gyro_rows_.end(); ++i)
    {
      GyroCompensaton* gyro = i->first;
      GyroRow& row = i->second;
      wxString val = row.pwm_text_->GetValue();
      long val2;
      if (val.ToLong(&val2))
      {
        row.pwm_slider_->SetValue(val2);
        gyro->modifier10_ = val2;
      }
    }
    
    SendGyroUpdate();
  }

private:
  GyroRowLibrary gyro_rows_;
  ros::Publisher gyro_pitch_pub_;
  ros::Publisher gyro_roll_pub_;
  
  void AddGyroRows(GyroCompensatonList& gyro_compensation_,
                   wxFlexGridSizer *sizer)
  {
    for (size_t i=0; i < gyro_compensation_.size(); i++)
    {
      GyroRow new_row;
                  
      wxString label = wxString::Format(_("%S"),
                                 gyro_compensation_[i].joint_name_.c_str());
      new_row.label_ = new wxStaticText(this, -1, label, wxPoint(0, 0), wxSize(100, -1),
                                        wxALIGN_LEFT | wxST_NO_AUTORESIZE);                                 
      new_row.pwm_slider_ = new wxSlider(this, ID_Slider, gyro_compensation_[i].modifier10_ , 
                                          -100, 100, wxPoint(50, 0),
                                          wxSize(400, -1), wxSL_HORIZONTAL);
      label = wxString::Format(_("%d"), gyro_compensation_[i].modifier10_);                            
      new_row.pwm_text_ = new wxTextCtrl(this, ID_Text, label, wxPoint(455, 0),
                                          wxSize(45, 20), wxTE_PROCESS_ENTER);
                                          
      sizer->Add(new_row.label_, 1, wxEXPAND, 0);
      sizer->Add(new_row.pwm_slider_ , 10, wxEXPAND, 0);
      sizer->Add(new_row.pwm_text_, 0, 0, 0); 
      
      gyro_rows_[&gyro_compensation_[i]] = new_row;
    } 
  }
  
  void PublishGyros(GyroCompensatonList& gyro_compensation_,
                    ros::Publisher& gyro_pub_)
  {
    for (size_t i=0; i < gyro_compensation_.size(); i++)
    {
      roboard_servos::SetGyroCompensation msg;
      msg.joint_name = gyro_compensation_[i].joint_name_;
      msg.modifier10 = gyro_compensation_[i].modifier10_;
      gyro_pub_.publish(msg);
    }
  }
  
  void SendGyroUpdate()
  {
    PublishGyros(gyro_pitch_compensation_, gyro_pitch_pub_);
    PublishGyros(gyro_roll_compensation_, gyro_roll_pub_);
  }
};

class GyroFrame: public wxFrame
{
public:
  GyroFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
           : wxFrame(NULL, -1, title, pos, size)
  {
    wxMenu *menuFile = new wxMenu;

    menuFile->Append(ID_About, _("&About..."));
    menuFile->AppendSeparator();
    menuFile->Append(ID_OpenFile, _("&Open..."));
    menuFile->Append(ID_SaveFile, _("&Save File..."));
    menuFile->AppendSeparator();
    menuFile->Append(ID_Quit, _("E&xit"));

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, _("&File"));

    SetMenuBar(menuBar);

    CreateStatusBar();
    SetStatusText(_("..."));
    
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(new GyroPanel(this), 0, wxEXPAND, 0);
    sizer->SetSizeHints(this);
    SetSizer(sizer);
    
    Connect(ID_Quit, wxEVT_COMMAND_MENU_SELECTED,
            (wxObjectEventFunction) &GyroFrame::OnQuit);
    Connect(ID_About, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(GyroFrame::OnAbout));
    Connect(ID_SaveFile, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(GyroFrame::OnSaveFile));
   
  }

  void OnQuit(wxCommandEvent& event)
  {
    Close(true);
  }

  void OnAbout(wxCommandEvent& event)
  {
    wxMessageBox(_("Use this application to adjust the gyro compensation of your robot"),
                 _("About Gyro GUI"), wxOK | wxICON_INFORMATION, this);
  }
  
  
  void OnSaveFile(wxCommandEvent& event)
  {
    wxFileDialog dlg(this, _("Choose a file"), _(""), _(""), _("*.xml"),
                     wxSAVE | wxOVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_OK)
    {
      std::string str(dlg.GetPath().mb_str());
      
      //servos_.saveURDFfile(str);
    }
  }

};

class GyroGUI: public wxApp
{
public:
  virtual bool OnInit()
  {
    // create our own copy of argv, with regular char*s.
    char** local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
      local_argv_[i] = strdup(wxString(argv[i]).mb_str());
    ros::init(argc, local_argv_, "gyro_gui");
    n_.reset(new ros::NodeHandle);
  
    GyroFrame *frame = new GyroFrame(_("Adjust Gyro Compensation"), wxPoint(0, 0),
                                     wxSize(500, 200));
                       
    frame->Show(true);
    SetTopWindow(frame);
    return true;
  }
};

} // namespace roboard_servos

IMPLEMENT_APP(roboard_servos::GyroGUI)


