#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStringListModel>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  QStringList motion_names;
  for (std::map<std::string, veltrobot_movement::Motion>::iterator i = pose_manager_.motions_.begin();
       i != pose_manager_.motions_.end(); ++i)
  {
    const std::string& motion_name = i->first;
    //const veltrobot_movement::Motion& motion_object = i->second;
    motion_names << QString::fromStdString(motion_name);
  }
  //QAbstractItemModel *motion_model = new QStringListModel(motion_names);
  motions_model_ = new QStringListModel(motion_names);
  ui->listView_motions->setModel(motions_model_);

  QStringList pose_names;
  for (std::map<std::string, veltrobot_movement::Pose>::iterator i = pose_manager_.poses_.begin();
       i != pose_manager_.poses_.end(); ++i)
  {
    const std::string& pose_name = i->first;
    //const veltrobot_movement::Pose& pose_object = i->second;
    pose_names << QString::fromStdString(pose_name);
  }
  //QAbstractItemModel *pose_model = new QStringListModel(pose_names);
  poses_model_ = new QStringListModel(pose_names);
  ui->listView_poses->setModel(poses_model_);

  connect(ui->listView_motions->selectionModel(),
          SIGNAL (selectionChanged ( const QItemSelection &, const QItemSelection & )),
          this,
          SLOT   (_on_listView_motions_selectionChanged ( const QItemSelection &, const QItemSelection &)));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::_on_listView_motions_selectionChanged ( const QItemSelection & selected, const QItemSelection & deselected)
{
  //if (!selected.size())
  //  return;
  const QItemSelectionRange& selected_range = selected.at(0);

  QString selected_motion_name_q = motions_model_->stringList()[selected_range.top()];
  std::string selected_motion_name = selected_motion_name_q.toStdString();

  veltrobot_movement::Motion& selected_motion = pose_manager_.motions_[selected_motion_name];

  QItemSelection selection;
  int first_index = poses_model_->stringList().size();
  int last_index = 0;
  for (std::map<std::string, veltrobot_movement::MotionPhase>::iterator i = selected_motion.phases_.begin();
       i != selected_motion.phases_.end(); ++i)
  {
    //const std::string& phase_name = i->first;
    const veltrobot_movement::MotionPhase& phase = i->second;

    for (size_t j=0; j < phase.poses_.size(); j++)
    {
      QString phase_name = QString::fromStdString(phase.poses_.at(j));

      int pose_index = poses_model_->stringList().indexOf(phase_name);
      if (pose_index < first_index)
        first_index = pose_index;
      if (pose_index > last_index)
        last_index = pose_index;
      const QModelIndex& pose_index_q = poses_model_->index(pose_index);

      QItemSelectionRange range(pose_index_q);
      selection.push_back(range);
    }
  }

  QItemSelectionModel *selectionModel = ui->listView_poses->selectionModel();
  selectionModel->select(selection, QItemSelectionModel::ClearAndSelect);

  ui->listView_poses->scrollTo(poses_model_->index(last_index));
  ui->listView_poses->scrollTo(poses_model_->index(first_index));
}

void MainWindow::on_pushButton_newPose_clicked()
{
  static unsigned int num = 0;
  std::ostringstream os;
  os << "new_pose_" << ++num;
  std::string pose_name(os.str());

  veltrobot_movement::Pose new_pose;
  new_pose.name_ = pose_name;
  pose_manager_.poses_[pose_name] = new_pose;

  PoseWindow* w = new PoseWindow(pose_manager_, pose_name);
  pose_windows_[pose_name] = w;
  w->show();
}


void MainWindow::on_pushButton_editPose_clicked()
{
  QItemSelectionModel *selectionModel = ui->listView_poses->selectionModel();
  QModelIndexList selected_indexes = selectionModel->selectedRows();

  for (int i=0; i < selected_indexes.size(); i++)
  {
    int selected_index = selected_indexes[i].row();
    QString pose_name_q = poses_model_->stringList().at(selected_index);
    std::string pose_name = pose_name_q.toStdString();

    PoseWindow* w = new PoseWindow(pose_manager_, pose_name);
    pose_windows_[pose_name] = w;
    w->show();
  }
}
