#include <map>
#include <vector>

#include <wx/wx.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include "servo.h"

namespace roboard_servos
{

ros::NodeHandlePtr n_;
ServoLibrary       servos_;

enum
{
  ID_Quit = 1,
  ID_OpenFile,
  ID_OpenParam,
  ID_SaveFile,
  ID_About,
  ID_Slider,
  ID_Text
};

struct ServoRow
{
  wxStaticText* label_;
  wxSlider *pwm_slider_;
  wxTextCtrl *pwm_text_;  
  //wxTextEntry *pwm_text_;
};
typedef std::map<Servo*, ServoRow> ServoRowLibrary;

class TrimPanel : public wxPanel
{
public:
  TrimPanel(wxFrame *parent)
           : wxPanel(parent, wxID_ANY)
  {
    servos_.openURDFparam();
    if (servos_.getUsedChannels() == 0)
      ROS_ERROR("Robot Description contains no servo channels");  
         
    wxFlexGridSizer *sizer = new wxFlexGridSizer(3);
    sizer->AddGrowableCol(0);
    sizer->AddGrowableCol(1);  
      
    ServoLibrary::iterator i = servos_.begin();
    for(; i != servos_.end(); ++i)
    {
      //std::string& joint_name = i->first;
      Servo& servo = i->second;
      
      if (servo.channel_ != -1)
      {
        ServoRow new_row;
        
        wxString label = wxString::Format(_("CH%d"), servo.channel_);
        new_row.label_ = new wxStaticText(this, -1, label, wxPoint(0, 0), wxSize(50, -1),
                                          wxALIGN_LEFT | wxST_NO_AUTORESIZE);  
        int pwm_range = (servo.max_pwm_ - servo.min_pwm_) / 2;                                 
        new_row.pwm_slider_ = new wxSlider(this, ID_Slider, servo.trim_pwm_, 
                                           -pwm_range, pwm_range, wxPoint(50, 0),
                                           wxSize(400, -1), wxSL_HORIZONTAL);
        label = wxString::Format(_("%d"), servo.trim_pwm_);                            
        new_row.pwm_text_ = new wxTextCtrl(this, ID_Text, label, wxPoint(455, 0),
                                            wxSize(45, 20), wxTE_PROCESS_ENTER);
                                            
        sizer->Add(new_row.label_, 1, wxEXPAND, 0);
        sizer->Add(new_row.pwm_slider_ , 10, wxEXPAND, 0);
        sizer->Add(new_row.pwm_text_, 0, 0, 0); 
        
        servo_rows_[&servo] = new_row;
      }
    }  
    
    sizer->SetSizeHints(this);
    SetSizer(sizer);
                          
    Connect(ID_Slider, wxEVT_COMMAND_SLIDER_UPDATED, 
            wxScrollEventHandler(TrimPanel::OnScroll));                       
    Connect(ID_Text, wxEVT_COMMAND_TEXT_ENTER,
            wxCommandEventHandler(TrimPanel::OnEnter)); 
            
    joint_states_pub_ = n_->advertise<sensor_msgs::JointState>("/joint_states", 10);
    update_trim_pub_ = n_->advertise<std_msgs::Bool>("/trim_updated", 10);  
  }

  void OnScroll(wxScrollEvent& event)
  {
    ServoRowLibrary::iterator i = servo_rows_.begin();
    for(; i != servo_rows_.end(); ++i)
    {
      Servo* servo = i->first;
      ServoRow& row = i->second;    
      int val = row.pwm_slider_->GetValue();
      row.pwm_text_->SetValue(wxString::Format(_("%d"), val));
      servo->trim_pwm_ = val;
    }
    
    servos_.saveURDFparam();
    SendJointStateUpdate();
  }
  
  void OnEnter(wxCommandEvent& event)
  {
    ServoRowLibrary::iterator i = servo_rows_.begin();
    for(; i != servo_rows_.end(); ++i)
    {
      Servo* servo = i->first;
      ServoRow& row = i->second;
      wxString val = row.pwm_text_->GetValue();
      long val2;
      if (val.ToLong(&val2))
      {
        row.pwm_slider_->SetValue(val2);
        servo->trim_pwm_ = val2;
      }
    }
    
    servos_.saveURDFparam();
    SendJointStateUpdate();
  }

private:
  ServoRowLibrary servo_rows_;
  //ServoLibrary    servos_;
  ros::Publisher  joint_states_pub_;
  ros::Publisher  update_trim_pub_;
  
  void SendJointStateUpdate()
  {
    std_msgs::Bool b;
    update_trim_pub_.publish(b);
  
    sensor_msgs::JointState  js;
    ServoLibrary::iterator i = servos_.begin();
    for(; i != servos_.end(); ++i)
    {
      std::string joint_name = i->first;
      //Servo& servo = i->second;  
      js.name.push_back(joint_name);
      js.position.push_back(0.0);     
      js.velocity.push_back(10);
    }
    joint_states_pub_.publish(js); 
  }
};

class TrimFrame: public wxFrame
{
public:
  TrimFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
           : wxFrame(NULL, -1, title, pos, size)
  {
    wxMenu *menuFile = new wxMenu;

    menuFile->Append(ID_About, _("&About..."));
    menuFile->AppendSeparator();
    menuFile->Append(ID_OpenParam, _("&Open from &Parameter..."));
    menuFile->Append(ID_OpenFile, _("&Open..."));
    menuFile->Append(ID_SaveFile, _("&Save File..."));
    menuFile->AppendSeparator();
    menuFile->Append(ID_Quit, _("E&xit"));

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, _("&File"));

    SetMenuBar(menuBar);

    CreateStatusBar();
    SetStatusText(_("Move the sliders to adjust trim positions"));
    
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(new TrimPanel(this), 0, wxEXPAND, 0);
    sizer->SetSizeHints(this);
    SetSizer(sizer);
    
    Connect(ID_Quit, wxEVT_COMMAND_MENU_SELECTED,
            (wxObjectEventFunction) &TrimFrame::OnQuit);
    Connect(ID_About, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(TrimFrame::OnAbout));
    Connect(ID_SaveFile, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(TrimFrame::OnSaveFile));
    Connect(ID_OpenFile, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(TrimFrame::OnOpenFile));   
    Connect(ID_OpenParam, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(TrimFrame::OnOpenParam));   
  }

  void OnQuit(wxCommandEvent& event)
  {
    Close(true);
  }

  void OnAbout(wxCommandEvent& event)
  {
    wxMessageBox(_("Use this application to adjust the trim of your robot"),
                 _("About Trim GUI"), wxOK | wxICON_INFORMATION, this);
  }
  
  void OnOpenFile(wxCommandEvent& event)
  {

  }
  
  void OnOpenParam(wxCommandEvent& event)
  {

  }
  
  void OnSaveFile(wxCommandEvent& event)
  {
    wxFileDialog dlg(this, _("Choose a file"), _(""), _(""), _("*.xml"),
                     wxSAVE | wxOVERWRITE_PROMPT);
    if (dlg.ShowModal() == wxID_OK)
    {
      std::string str(dlg.GetPath().mb_str());
      servos_.saveURDFfile(str);
    }
  }

};

class TrimGUI: public wxApp
{
public:
  virtual bool OnInit()
  {
    // create our own copy of argv, with regular char*s.
    char** local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
      local_argv_[i] = strdup(wxString(argv[i]).mb_str());
    ros::init(argc, local_argv_, "trim_gui");
    n_.reset(new ros::NodeHandle);
  
    TrimFrame *frame = new TrimFrame(_("Adjust Trim"), wxPoint(0, 0), wxSize(500, 200));
                       
    frame->Show(true);
    SetTopWindow(frame);
    return true;
  }
  
private:
//  ros::NodeHandlePtr n_;
};

} // namespace roboard_servos

IMPLEMENT_APP(roboard_servos::TrimGUI)

