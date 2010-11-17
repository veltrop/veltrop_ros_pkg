#include "jointwidget.h"
#include "ui_jointwidget.h"

#define PI 3.14159265359f

JointWidget::JointWidget(float position, const std::string& name, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JointWidget)
{
    ui->setupUi(this);
    ui->comboBox->setEditText(QString::fromStdString(name));
    ui->dial->setValue(position*1000);
    ui->doubleSpinBox_position->setValue(position);
}

JointWidget::~JointWidget()
{
    delete ui;
}
