#ifndef JOINTWIDGET_H
#define JOINTWIDGET_H

#include <QWidget>

namespace Ui {
    class JointWidget;
}

class JointWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JointWidget(float position, const std::string& name, QWidget *parent = 0);
    ~JointWidget();

    void setPosition(float position);
    float getPosition() const;
    std::string getName() const;
    bool enableCapture() const;

private:
    Ui::JointWidget *ui;
};

#endif // JOINTWIDGET_H
