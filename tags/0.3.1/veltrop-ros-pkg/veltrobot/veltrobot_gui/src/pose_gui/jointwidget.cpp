#include "jointwidget.h"
#include "ui_jointwidget.h"

#define PI 3.14159265359f

JointWidget::JointWidget(float position, const std::string& name, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JointWidget)
{
    ui->setupUi(this);
    ui->comboBox->setEditText(QString::fromStdString(name));
    ui->doubleSpinBox_position->setValue(position);
}

JointWidget::~JointWidget()
{
    delete ui;
}

void JointWidget::setPosition(float position)
{
  ui->doubleSpinBox_position->setValue(position);
}

float JointWidget::getPosition() const
{
 return ui->doubleSpinBox_position->value();
}

std::string JointWidget::getName() const
{
  return ui->comboBox->currentText().toStdString();
}

bool JointWidget::enableCapture() const
{
  return (ui->checkBox_capture->checkState() == Qt::Checked);
}
