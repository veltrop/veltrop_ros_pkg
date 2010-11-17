#include <QtGui/QApplication>
#include "mainwindow.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pose_gui");
  QApplication a(argc, argv);

  MainWindow w;
  w.show();

  return a.exec();
}
