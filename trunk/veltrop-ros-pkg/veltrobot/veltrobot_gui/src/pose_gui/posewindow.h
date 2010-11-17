#ifndef POSEWINDOW_H
#define POSEWINDOW_H

#include <QMainWindow>
#include <veltrobot_movement/posemanager.h>

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
};

#endif // POSEWINDOW_H
