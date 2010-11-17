#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringListModel>
#include "posewindow.h"
#include <map>
#include <veltrobot_movement/posemanager.h>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    std::map <std::string, PoseWindow*> pose_windows_;
    veltrobot_movement::PoseManager pose_manager_;
    Ui::MainWindow *ui;
    QStringListModel* motions_model_;
    QStringListModel* poses_model_;

private slots:
    void on_pushButton_editPose_clicked();
    void _on_listView_motions_selectionChanged( const QItemSelection & selected, const QItemSelection & deselected);
    void on_pushButton_newPose_clicked();
};

#endif // MAINWINDOW_H
