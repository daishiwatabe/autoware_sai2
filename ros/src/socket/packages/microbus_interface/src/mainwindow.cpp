#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->bt_emergency_clear, SIGNAL(clicked()), this, SLOT(publish_emergency_clear()));

    nh_ = nh;  private_nh_ = p_nh;
    pub_emergency_clear_ = nh_.advertise<std_msgs::Empty>("/microbus/emergency_reset", 1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::publish_emergency_clear()
{
    //std::cout << "aaa" << std::endl;
    std_msgs::Empty msg;
    pub_emergency_clear_.publish(msg);
}
