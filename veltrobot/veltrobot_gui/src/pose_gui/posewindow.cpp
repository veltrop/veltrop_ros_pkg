#include "posewindow.h"
#include "ui_posewindow.h"
#include "jointwidget.h"

#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <veltrobot_msgs/CapturePose.h>

// TODO: constructor for "new pose" which will read default joint names from
//       servos.yaml.

bool PoseWindow::entered_capture_ = false;

PoseWindow::PoseWindow(veltrobot_movement::PoseManager& pose_manager,
                       const std::string& pose_name,
                       QWidget *parent) :
    QMainWindow(parent),
    pose_name_(pose_name),
    pose_manager_(pose_manager),
    ui(new Ui::PoseWindow),
    row_(0), col_(0)
{
    ui->setupUi(this);

    ros::NodeHandle n;
    joint_ics_pub_ = n.advertise<std_msgs::Int16>("/servo_command", 10);
    capture_pose_client_ = n.serviceClient<veltrobot_msgs::CapturePose>("/capture_pose");
    joint_state_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states_alternate", 1);

    populatePoseList();

    OpenPose(pose_name_);
}

PoseWindow::~PoseWindow()
{
    delete ui;
}

void PoseWindow::populatePoseList()
{
  ui->comboBox_poseName->clear();

  QStringList pose_names;
  for (std::map<std::string, veltrobot_movement::Pose>::iterator i = pose_manager_.poses_.begin();
  i != pose_manager_.poses_.end(); ++i)
  {
    const std::string& pose_name = i->first;
    //const veltrobot_movement::Pose& pose_object = i->second;
    pose_names << QString::fromStdString(pose_name);
  }

  ui->comboBox_poseName->insertItems(0, pose_names);
}

void PoseWindow::OpenPose(const std::string& pose_name)
{
  pose_manager_.poses_[pose_name].name_ = pose_name;
  OpenPose(pose_manager_.poses_[pose_name]);
}

void PoseWindow::OpenPose(veltrobot_movement::Pose& pose)
{
  //pose_ = pose;
  this->setWindowTitle(QString::fromStdString(pose.name_));

  for (size_t i=0; i < joint_widgets_.size(); i++)
  {
    QWidget* w;
    w = joint_widgets_[i];
    ui->gridLayout->removeWidget(w);
    delete w;
  }
  joint_widgets_.clear();
  row_ = 0;
  col_ = 0;

  //ui->lineEdit_name->setText(QString::fromStdString(pose.name_));
  ui->comboBox_poseName->setEditText(QString::fromStdString(pose.name_));

  ui->spinBox_duration->setValue(pose.duration_);

  ui->gridLayout->setColumnMinimumWidth(0, 370);
  ui->gridLayout->setColumnMinimumWidth(1, 370);
  for (std::map<std::string, float>::iterator i = pose.positions_.begin();
       i != pose.positions_.end(); ++i)
  {
    const std::string& joint_name = i->first;
    float joint_position = i->second;

    ui->gridLayout->setRowMinimumHeight(row_,26);
    JointWidget* jw = new JointWidget(joint_position, joint_name);
    //joint_widgets_[joint_name] = jw;
    joint_widgets_.push_back(jw);

    ui->gridLayout->addWidget(jw, row_, col_);
    if (++col_==2)
    {
      col_ = 0;
      row_++;
    }
  }
}

void PoseWindow::on_pushButton_capture_clicked()
{
  veltrobot_msgs::CapturePose srv;

  //for (std::map<std::string, JointWidget*>::iterator i = joint_widgets_.begin();
  //     i != joint_widgets_.end(); ++i)
  //{
  //  const std::string& joint_name = i->first;
  //  srv.request.requestedJointNames.push_back(joint_name);
  //}
  for (size_t i=0; i < joint_widgets_.size(); i++)
  {
    srv.request.requestedJointNames.push_back(joint_widgets_[i]->getName());
  }


  if (capture_pose_client_.call(srv))
  {
    for (size_t i=0; i < srv.response.jointNames.size(); i++)
    {
      //joint_widgets_[srv.response.jointNames[i]]->setPosition(srv.response.jointPositions[i]);
      for (size_t j=0; j < joint_widgets_.size(); j++)
      {
        if (joint_widgets_[j]->getName() == srv.response.jointNames[i] &&
            joint_widgets_[j]->enableCapture())
          joint_widgets_[j]->setPosition(srv.response.jointPositions[i]);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service capture_pose");
  }
}

void PoseWindow::on_pushButton_rest_clicked()
{
  std_msgs::Int16 msg;
  msg.data = 0;
  joint_ics_pub_.publish(msg);
  msg.data = 100;
  joint_ics_pub_.publish(msg);

  entered_capture_ = true;
}

void PoseWindow::on_pushButton_unrest_clicked()
{
  std_msgs::Int16 msg;
  msg.data = 101;
  joint_ics_pub_.publish(msg);

  entered_capture_ = false;
}

void PoseWindow::on_pushButton_apply_clicked()
{
  if (entered_capture_)
    on_pushButton_unrest_clicked();
  usleep(100);

  sensor_msgs::JointState js;
  int duration = ui->spinBox_duration->value();

  //for (std::map<std::string, JointWidget*>::iterator i = joint_widgets_.begin();
  //     i != joint_widgets_.end(); ++i)
  //{
  //  const std::string& joint_name = i->first;
  //  const JointWidget* joint = i->second;

  //  js.name.push_back(joint_name);
  //  js.position.push_back(joint->getPosition());
  //  js.velocity.push_back(duration);
  //}

  for (size_t i=0; i < joint_widgets_.size(); i++)
  {
    js.name.push_back(joint_widgets_[i]->getName());
    js.position.push_back(joint_widgets_[i]->getPosition());
    js.velocity.push_back(duration);
  }

  joint_state_pub_.publish(js);
}


void PoseWindow::on_pushButton_save_clicked()
{
  //pose_name_ = ui->lineEdit_name->text().toStdString();
  pose_name_ = ui->comboBox_poseName->currentText().toStdString();
  veltrobot_movement::Pose& pose = pose_manager_.poses_[pose_name_];

  pose.duration_ = ui->spinBox_duration->value();
  pose.name_ = pose_name_;
  this->setWindowTitle(QString::fromStdString(pose.name_));

  //for (std::map<std::string, JointWidget*>::iterator i = joint_widgets_.begin();
  //     i != joint_widgets_.end(); ++i)
  //{
  //  const std::string& joint_name = i->first;
  //  const JointWidget* joint = i->second;

  //  pose.positions_[joint_name] = joint->getPosition();
  //}
  for (size_t i=0; i < joint_widgets_.size(); i++)
  {
    pose.positions_[joint_widgets_[i]->getName()] = joint_widgets_[i]->getPosition();
  }

  std::string filename = pose_manager_.getPosePath(pose_name_);
  pose.saveXML(filename);

  pose_manager_.reload();
  //populatePoseList();
  //OpenPose(pose_name_);
}

void PoseWindow::on_pushButton_add_clicked()
{
  static unsigned int num = 0;
  std::ostringstream os;
  os << "new_joint_" << ++num;
  std::string joint_name(os.str());

  ui->gridLayout->setRowMinimumHeight(row_,26);
  JointWidget* jw = new JointWidget(0, joint_name);
  joint_widgets_.push_back(jw);
  ui->gridLayout->addWidget(jw, row_, col_);
  if (++col_==2)
  {
    col_ = 0;
    row_++;
  }
}

void PoseWindow::on_comboBox_poseName_currentIndexChanged(QString poseName)
{
  OpenPose(poseName.toStdString());
}
