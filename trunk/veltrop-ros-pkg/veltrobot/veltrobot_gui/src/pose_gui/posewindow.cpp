#include "posewindow.h"
#include "ui_posewindow.h"
#include "jointwidget.h"

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

    ui->gridLayout->addWidget(jw, row, col);
    if (++col==2)
    {
      col = 0;
      row++;
    }
  }
}
