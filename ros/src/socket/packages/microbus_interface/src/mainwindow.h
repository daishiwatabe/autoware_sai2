#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_emergency_clear_;
private slots:
    void publish_emergency_clear();
};

#endif // MAINWINDOW_H
