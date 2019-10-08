#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "microbus_interface");
    ros::NodeHandle nh, private_nh("~");

    MainWindow w(nh, private_nh);
    w.show();

    //return a.exec();

    ros::Rate loop_rate(100);
    while (ros::ok()){
      ros::spinOnce();
      a.processEvents();

      w.window_updata();

      loop_rate.sleep();
    }
}
