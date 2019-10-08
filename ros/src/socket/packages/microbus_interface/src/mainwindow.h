#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent = nullptr);
    ~MainWindow();

    void window_updata();
private:
    Ui::MainWindow *ui;

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher pub_unlock_;//デバイス立ち上がり時のLOCKを解除
    ros::Publisher pub_drive_mode_, pub_steer_mode_;//autoモードとmanualモードのチェンジ
    ros::Publisher pub_drive_control_;//driveのコントロールモード(velocity操作とstroke操作の切り替え)
    ros::Publisher pub_drive_input_, pub_steer_input_;//programモード時の自動入力と手動入力の切り替え

    ros::Subscriber sub_can501_, sub_can502_;//マイクロバスcanのID501,502

    void callbackCan501(const autoware_can_msgs::MicroBusCan501 &msg);//マイコン応答ID501
    void callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg);//マイコン応答ID502

    autoware_can_msgs::MicroBusCan501 can501_;//マイコン応答ID501

    QPalette palette_drive_mode_ok_, palette_steer_mode_ok_;//autoモード表示テキストボックスのバックグラウンドカラーOK
    QPalette palette_drive_mode_error_, palette_steer_mode_error_;//autoモード表示テキストボックスのバックグラウンドカラーerror
private slots:
    void publish_emergency_clear();
    void publish_Dmode_manual();
    void publish_Dmode_program();
    void publish_Smode_manual();
    void publish_Smode_program();
    void publish_Dmode_velocity();
    void publish_Dmode_stroke();
    void publish_Dmode_input_direct();
    void publish_Dmode_input_auto();
    void publish_Smode_input_direct();
    void publish_Smode_input_auto();
};

#endif // MAINWINDOW_H
