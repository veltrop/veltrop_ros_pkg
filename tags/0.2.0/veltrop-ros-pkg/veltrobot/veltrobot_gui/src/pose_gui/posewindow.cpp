#include "posewindow.h"
#include "ui_posewindow.h"
#include "jointwidget.h"

#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <veltrobot_msgs/CapturePose.h>

// TODO: constructor for "new pose" which will read default joint names from
//       servos.yaml.

PoseWindow::PoseWindow(veltrobot_movement::PoseManager& pose_manager,
                       const std::string& pose_name,
                       QWidget *parent) :
    QMainWindow(parent),
    pose_name_(pose_name),
    pose_manager_(pose_manager),
    ui(new Ui::PoseWindow)
{
    ui->setupUi(this);

    OpenPose(pose_name_);

    ros::NodeHandle n;
    joint_ics_pub_ = n.advertise<std_msgs::Int16>("/servo_command", 1);
    capture_pose_client_ = n.serviceClient<veltrobot_msgs::CapturePose>("/capture_pose");
    joint_state_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

PoseWindow::~PoseWindow()
{
    delete ui;
}

void PoseWindow::OpenPose(const std::string& pose_name)
{
  OpenPose(pose_manager_.poses_[pose_name]);
}

void PoseWindow::OpenPose(veltrobot_movement::Pose& pose)
{
  this->setWindowTitle(QString::fromStdString(pose.name_));
  joint_widgets_.clear();

  ui->spinBox_duration->setValue(pose.duration_);

  ui->gridLayout->setColumnMinimumWidth(0, 392);
  ui->gridLayout->setColumnMinimumWidth(1, 392);
  int col=0,row=0;
  for (std::map<std::string, float>::iterator i = pose.positions_.begin();
       i != pose.positions_.end(); ++i)
  {
    const std::string& joint_name = i->first;
    float joint_position = i->second;

    ui->gridLayout->setRowMinimumHeight(row,61);
    JointWidget* jw = new JointWidget(joint_position, joint_name);
    joint_widgets_[joint_name] = jw;

    ui->gridLayout->addWidget(jw, row, col);
    if (++col==2)
    {
      col = 0;
      row++;
    }
  }
}

void PoseWindow::on_pushButton_capture_clicked()
{
  veltrobot_msgs::CapturePose srv;

  for (std::map<std::string, JointWidget*>::iterator i = joint_widgets_.begin();
       i != joint_widgets_.end(); ++i)
  {
    const std::string& joint_name = i->first;
    srv.request.requestedJointNames.push_back(joint_name);
  }

  if (capture_pose_client_.call(srv))
  {
    for (size_t i=0; i < srv.response.jointNames.size(); i++)
    {
      joint_widgets_[srv.response.jointNames[i]]->setPosition(srv.response.jointPositions[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service capture_pose");
  }
}

void PoseWindow::on_pushButton_rest_clicked()
{
  static bool hit = false;
  std_msgs::Int16 msg;
  hit ? msg.data = 1 : msg.data = 0;
  hit = !hit;
  joint_ics_pub_.publish(msg);
}

void PoseWindow::on_pushButton_apply_clicked()
{
  sensor_msgs::JointState js;

  //js.name.push_back(joint_name);
  //js.position.push_back(position);
  //js.velocity.push_back(duration);

  joint_state_pub_.publish(js);
}
