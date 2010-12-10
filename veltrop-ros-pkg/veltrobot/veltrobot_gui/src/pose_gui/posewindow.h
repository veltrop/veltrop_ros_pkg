#ifndef POSEWINDOW_H
#define POSEWINDOW_H

#include <QMainWindow>
#include <veltrobot_movement/posemanager.h>
#include <ros/ros.h>
#include "jointwidget.h"

namespace Ui {
    class PoseWindow;
}

class PoseWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PoseWindow(veltrobot_movement::PoseManager& pose_manager,
                        const std::string& pose_name,
                        QWidget *parent = 0);
    ~PoseWindow();

    void OpenPose(const std::string& pose_name);
    void OpenPose(veltrobot_movement::Pose& pose);

private:
    std::string pose_name_;
    veltrobot_movement::PoseManager& pose_manager_;
    Ui::PoseWindow *ui;
    std::map<std::string, JointWidget*> joint_widgets_;

    ros::Publisher  joint_ics_pub_;
    ros::Publisher  joint_state_pub_;
    ros::ServiceClient capture_pose_client_;

private slots:
    void on_pushButton_apply_clicked();
    void on_pushButton_rest_clicked();
    void on_pushButton_capture_clicked();
};

#endif // POSEWINDOW_H
