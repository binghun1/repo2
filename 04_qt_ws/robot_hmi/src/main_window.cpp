/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_hmi/main_window.hpp"
#include <QDebug>
#include <sstream>
#include <QMessageBox>


using namespace pcl;
using namespace cv;
using namespace pcl::io;
using namespace std;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//int typeDef(int typeNum);

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

QByteArray MainWindow::QString2Hex(QString hexStr)
{
    QByteArray senddata;
    int hexdata, lowhexdata;
    int hexdatalen = 0;
    int len = hexStr.length();
    senddata.resize(len/2);
    char lstr, hstr;
    for(int i = 0; i < len; )
    {
        //将第一个不为' '的字符赋给hstr;
        hstr = hexStr[i].toLatin1();
        if(hstr == ' ')
        {
            i++;
            continue;
        }
        i++;
        //当i >= len时，跳出循环
        if(i >= len)
            break;
        //当i < len时，将下一个字符赋值给lstr;
        lstr = hexStr[i].toLatin1();
        //将hstr和lstr转换为0-15的对应数值
        hexdata = ConvertHexChar(hstr);
        lowhexdata = ConvertHexChar(lstr);
        //
        if((hexdata == 16) || (lowhexdata == 16))
            break;
        else
            hexdata = hexdata * 16 + lowhexdata;
        i++;
        senddata[hexdatalen] = (char)hexdata;
        hexdatalen++;
    }
    senddata.resize(hexdatalen);
    return senddata;
}

//将单个字符串转换为hex
//0-F -> 0-15
char MainWindow::ConvertHexChar(char c)
{
    if((c >= '0') && (c <= '9'))
        return c - 0x30;
    else if((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;//'A' = 65;
    else if((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;
    else
        return -1;
}

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui.widget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui.widget->GetInteractor(), ui.widget->GetRenderWindow());
    ui.widget->update();

    agv_image.create(360, 610, CV_8UC3);
    Vec3b color;  //设置的背景色
    color[0] = 255;//rng.uniform(0, 255);
    color[1] = 255;// rng.uniform(0, 255);
    color[2] = 255;
    for (size_t row=0; row<360; row++)
    {
        for (size_t col=0;col<610;col++)
        {
            //设置原图像中某点的BGR颜色值
            agv_image.at<Vec3b>(row, col) = Vec3b(color(0), color(1), color(2));
        }
    }

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();  //读取配置文件
    setWindowIcon(QIcon(":/images/icon.png"));   //图标
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);    //选择框选中时
    }
    connect(ui.quit_button,SIGNAL(clicked()),this,SLOT(quit_ui()));
    //拖动显示条
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_change(int)));
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    //方向键
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_k,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_bt,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_br,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));

    //init ui
    //速度仪表盘
    speed_x_dashBoard=new CCtrlDashBoard(ui.widget_speed_x,100,true);
    speed_y_dashBoard=new CCtrlDashBoard(ui.widget_speed_y,3,false);
    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect());
    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
    speed_x_dashBoard->setValue(0);  //仪表盘的默认数值
    speed_y_dashBoard->setValue(0);

    //connect,电量显示
    connect(&qnode,SIGNAL(speed_vel(float,float,float,float,float)),this,SLOT(slot_update_dashboard(float,float,float,float,float)));
    connect(&qnode,SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));

    ui.label_power_value->setText(QString::number(12.5).mid(0,5)+"V");
    ui.progressBar->setValue(100);

    //connect(ui.laser_btn,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_clicked()));
    connect(ui.point_open_btn,SIGNAL(clicked()),this,SLOT(slot_point_open()));
    connect(ui.point_save_btn,SIGNAL(clicked()),this,SLOT(slot_point_save()));
    connect(&qnode,SIGNAL(image_vel(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));

    //串口
    //连接信号和槽
    //connect(ui.openButton,SIGNAL(clicked()),this,SLOT(on_openButton_clicked()));
    connect(ui.search_serial_btn,SIGNAL(clicked()),this,SLOT(on_searchButton_clicked()));
    connect(ui.serialrec_clear_btn,SIGNAL(clicked()),this,SLOT(on_clearButton_clicked()));
    connect(&serial, &QSerialPort::readyRead, this, &MainWindow::serialPort_readyRead);
    //发送按键失能
    ui.sendButton->setEnabled(false);
    //波特率默认选择下拉第三项：9600
    ui.baudrateBox->setCurrentIndex(3);
    ui.dataBitsBox->setCurrentIndex(3);
    ui.ParityBox->setCurrentIndex(0);
    ui.stopBitsBox->setCurrentIndex(0);
    ui.speedBox->setCurrentIndex(0);

    //雷达
    connect(ui.laser_open_btn,SIGNAL(clicked()),this,SLOT(radar_pushbtn_click()));
    connect(ui.laser_close_btn,SIGNAL(clicked()),this,SLOT(radar_pushbtn_click()));

    //电机
    ui.motorpos_read_btn->setEnabled(false);
    ui.motor_forward_btn->setEnabled(false);
    ui.motor_reverses_btn->setEnabled(false);
    ui.motorpos_init_btn->setEnabled(false);
    connect(ui.motor_init_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motor_lock_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motor_offline_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motor_forward_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motor_reverses_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motor_stop_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motorpos_init_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(ui.motorpos_read_btn,SIGNAL(clicked()),this,SLOT(motor_pushbtn_click()));
    connect(&qnode,SIGNAL(motorAxis_vel(int)),this,SLOT(motorAxis_get(int)));

    //IMU
    ui.imu_open_btn->setEnabled(false);
    ui.imu_close_btn->setEnabled(false);
    connect(ui.imu_init_btn,SIGNAL(clicked()),this,SLOT(imu_pushbtn_click()));
    connect(ui.imu_open_btn,SIGNAL(clicked()),this,SLOT(imu_pushbtn_click()));
    connect(ui.imu_close_btn,SIGNAL(clicked()),this,SLOT(imu_pushbtn_click()));

    //轨迹显示
    connect(ui.agv_pose_btn,SIGNAL(clicked()),this,SLOT(agvPose_pushbtn_click()));
    connect(ui.agv_path_btn,SIGNAL(clicked()),this,SLOT(agvPath_pushbtn_click()));

    //时间：
    m_pLCD = new QLCDNumber(ui.lcdNumber_time);
    // 设置能显示的位数
    m_pLCD->setDigitCount(23);
    // 设置显示的模式为十进制
    m_pLCD->setMode(QLCDNumber::Dec);
    // 设置显示外观
    m_pLCD->setSegmentStyle(QLCDNumber::Flat);
    m_pLCD->resize(280,40);
    // 设置样式
    //m_pLCD->setStyleSheet("border: 1px solid green; color: green; background: silver");
    m_pLCD->show();
    //构建一个定时器，每隔一秒来定时刷新QLCDNumber中的内容
    QTimer *pTimer = new QTimer(this);
    // 设置定时间隔
    pTimer->setInterval(100);
    connect(pTimer, SIGNAL(timeout()), this, SLOT(onTimeOut()));
    // 启动定时器
    pTimer->start();

    //ros时间
    ui.lcdNumber_rostime->setEnabled(false);
    m_pLCD_ros = new QLCDNumber(ui.lcdNumber_rostime);
    // 设置能显示的位数
    m_pLCD_ros->setDigitCount(18);
    // 设置显示的模式为十进制
    m_pLCD_ros->setMode(QLCDNumber::Dec);
    // 设置显示外观
    m_pLCD_ros->setSegmentStyle(QLCDNumber::Flat);
    m_pLCD_ros->resize(280,40);
    // 设置样式
    //m_pLCD->setStyleSheet("border: 1px solid green; color: green; background: silver");
    m_pLCD_ros->show();

    //电机转角
    ui.lcdNumber_motor->setEnabled(false);
    motor_pLCD = new QLCDNumber(ui.lcdNumber_motor);
    // 设置能显示的位数
    motor_pLCD->setDigitCount(6);
    // 设置显示的模式为十进制
    motor_pLCD->setMode(QLCDNumber::Dec);
    // 设置显示外观
    motor_pLCD->setSegmentStyle(QLCDNumber::Flat);
    motor_pLCD->resize(280,40);
    // 设置样式
    //m_pLCD->setStyleSheet("border: 1px solid green; color: green; background: silver");
    motor_pLCD->show();

    //开始3D点云采集
    connect(ui.point3D_start_btn,SIGNAL(clicked()),this,SLOT(point_pushbtn_click()));
    connect(ui.point3D_stop_btn,SIGNAL(clicked()),this,SLOT(point_pushbtn_click()));

    //IMU
    connect(&qnode,SIGNAL(imu_vel(float,float,float,float,float,float,float,float,float)),this,SLOT(imuData_get(float,float,float,float,float,float,float,float,float)));
    imu_pLCD1 = new QLCDNumber(ui.lcdNumber_roll);
    imu_pLCD1->setDigitCount(5);  //设置所显示的位数
    imu_pLCD1->setMode(QLCDNumber::Dec);  //以十进制形式显示（默认）
    imu_pLCD1->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD1->resize(80,40);
    imu_pLCD1->show();

    imu_pLCD2 = new QLCDNumber(ui.lcdNumber_pitch);
    imu_pLCD2->setDigitCount(5);
    imu_pLCD2->setMode(QLCDNumber::Dec);
    imu_pLCD2->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD2->resize(80,40);
    imu_pLCD2->show();

    imu_pLCD3 = new QLCDNumber(ui.lcdNumber_yaw);
    imu_pLCD3->setDigitCount(5);
    imu_pLCD3->setMode(QLCDNumber::Dec);
    imu_pLCD3->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD3->resize(80,40);
    imu_pLCD3->show();

    imu_pLCD4 = new QLCDNumber(ui.lcdNumber_wx);
    imu_pLCD4->setDigitCount(5);
    imu_pLCD4->setMode(QLCDNumber::Dec);
    imu_pLCD4->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD4->resize(80,40);
    imu_pLCD4->show();

    imu_pLCD5 = new QLCDNumber(ui.lcdNumber_wy);
    imu_pLCD5->setDigitCount(5);
    imu_pLCD5->setMode(QLCDNumber::Dec);
    imu_pLCD5->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD5->resize(80,40);
    imu_pLCD5->show();

    imu_pLCD6 = new QLCDNumber(ui.lcdNumber_wz);
    imu_pLCD6->setDigitCount(5);
    imu_pLCD6->setMode(QLCDNumber::Dec);
    imu_pLCD6->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD6->resize(80,40);
    imu_pLCD6->show();

    imu_pLCD7 = new QLCDNumber(ui.lcdNumber_ax);
    imu_pLCD7->setDigitCount(5);
    imu_pLCD7->setMode(QLCDNumber::Dec);
    imu_pLCD7->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD7->resize(80,40);
    imu_pLCD7->show();

    imu_pLCD8 = new QLCDNumber(ui.lcdNumber_ay);
    imu_pLCD8->setDigitCount(5);
    imu_pLCD8->setMode(QLCDNumber::Dec);
    imu_pLCD8->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD8->resize(80,40);
    imu_pLCD8->show();

    imu_pLCD9 = new QLCDNumber(ui.lcdNumber_az);
    imu_pLCD9->setDigitCount(5);
    imu_pLCD9->setMode(QLCDNumber::Dec);
    imu_pLCD9->setSegmentStyle(QLCDNumber::Flat);
    imu_pLCD9->resize(80,40);
    imu_pLCD9->show();

    agv_pLCD_x = new QLCDNumber(ui.lcdNumber_agv_x);
    agv_pLCD_x->setDigitCount(5);
    agv_pLCD_x->setMode(QLCDNumber::Dec);
    agv_pLCD_x->setSegmentStyle(QLCDNumber::Flat);
    agv_pLCD_x->resize(160,40);
    agv_pLCD_x->show();

    agv_pLCD_y = new QLCDNumber(ui.lcdNumber_agv_y);
    agv_pLCD_y->setDigitCount(5);
    agv_pLCD_y->setMode(QLCDNumber::Dec);
    agv_pLCD_y->setSegmentStyle(QLCDNumber::Flat);
    agv_pLCD_y->resize(160,40);
    agv_pLCD_y->show();

    agv_pLCD_theta = new QLCDNumber(ui.lcdNumber_agv_theta);
    agv_pLCD_theta->setDigitCount(5);
    agv_pLCD_theta->setMode(QLCDNumber::Dec);
    agv_pLCD_theta->setSegmentStyle(QLCDNumber::Flat);
    agv_pLCD_theta->resize(160,40);
    agv_pLCD_theta->show();


    //pcl点云
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    connect(&qnode,SIGNAL(pcl_vel()),this,SLOT(pcl3D_show()));
    //pcl拖动显示条
    connect(ui.horizontalSlider_red,SIGNAL(valueChanged(int)),this,SLOT(slot_red_value_change(int)));
    connect(ui.horizontalSlider_blue,SIGNAL(valueChanged(int)),this,SLOT(slot_blue_value_change(int)));
    connect(ui.horizontalSlider_green,SIGNAL(valueChanged(int)),this,SLOT(slot_green_value_change(int)));
    connect(ui.horizontalSlider_size,SIGNAL(valueChanged(int)),this,SLOT(slot_size_value_change(int)));
}

void MainWindow::slot_red_value_change(int value)
{
    background_red=value;
    ui.label_red->setText(QString::number(value));
}

void MainWindow::slot_green_value_change(int value)
{
    background_green=value;
    ui.label_green->setText(QString::number(value));
}

void MainWindow::slot_blue_value_change(int value)
{
    background_blue=value;
    ui.label_blue->setText(QString::number(value));
}

void MainWindow::slot_size_value_change(int value)
{
    point_size=value;
    ui.label_size->setText(QString::number(value));
}

void MainWindow::imuData_get(float a0,float a1,float a2,float a3,float a4,float a5,float a6,float a7,float a8)
{
    imu_pLCD1->display(a0);
    imu_pLCD2->display(a1);
    imu_pLCD3->display(a2);
    imu_pLCD4->display(a3);
    imu_pLCD5->display(a4);
    imu_pLCD6->display(a5);
    imu_pLCD7->display(a6);
    imu_pLCD8->display(a7);
    imu_pLCD9->display(a8);
}

void MainWindow::onTimeOut()
{
    //ros::NodeHandle n;
    // 获取系统当前时间
    QDateTime dateTime = QDateTime::currentDateTime();
    double double_time=0;
    if(qnode.ros_init_flag)
    {
        ui.lcdNumber_rostime->setEnabled(true);
        qnode.getTime(double_time);
    }
    // 显示的内容
    m_pLCD->display(dateTime.toString("yyyy-MM-dd HH:mm:ss.zzz"));
    m_pLCD_ros->display(double_time);

}

//串口
void MainWindow::serialPort_readyRead()
{
    //从接收缓冲区中读取数据
    QByteArray buffer = serial.readAll();
    //ui->textEdit->setText(temp.toHex());
    //从界面中读取以前收到的数据
    QString new_recv = ui.recvTextEdit->toPlainText();
    QString recv = QString(buffer.toHex());
    new_recv += " ";
    // 添加空格
    for(int i=0; i<recv.length(); i+=2)
    {
        QString st = recv.mid(i,2);
        new_recv += st;
        new_recv += " ";
    }
    //清空以前的显示
    ui.recvTextEdit->clear();
    //重新显示
    ui.recvTextEdit->append(new_recv);
}

void MainWindow::on_searchButton_clicked()
{
    ui.portNameBox->clear();
    //通过QSerialPortInfo查找可用串口
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        ui.portNameBox->addItem(info.portName());
    }
}

void MainWindow::on_openButton_clicked()
{
    //qDebug()<<ui.openButton->text();
    if(ui.openButton->text()==QString("打开串口"))
    {
        //设置串口名
        serial.setPortName(ui.portNameBox->currentText());
        //设置波特率
        serial.setBaudRate(ui.baudrateBox->currentText().toInt());
        //设置数据位数
        switch(ui.dataBitsBox->currentIndex())
        {
        case 0: serial.setDataBits(QSerialPort::Data5); break;
        case 1: serial.setDataBits(QSerialPort::Data6); break;
        case 2: serial.setDataBits(QSerialPort::Data7); break;
        case 3: serial.setDataBits(QSerialPort::Data8); break;
        default: break;
        }
        //设置奇偶校验
        switch(ui.ParityBox->currentIndex())
        {
        case 0: serial.setParity(QSerialPort::NoParity); break;
        default: break;
        }
        //波特率
        switch(ui.baudrateBox->currentIndex())
        {
        case 0: serial.setBaudRate(QSerialPort::Baud1200); break;
        case 1: serial.setBaudRate(QSerialPort::Baud2400); break;
        case 2: serial.setBaudRate(QSerialPort::Baud4800); break;
        case 3: serial.setBaudRate(QSerialPort::Baud9600); break;
        case 4: serial.setBaudRate(QSerialPort::Baud19200); break;
        case 5: serial.setBaudRate(QSerialPort::Baud115200); break;
        default: break;
        }
        //设置停止位
        switch(ui.stopBitsBox->currentIndex())
        {
        case 0: serial.setStopBits(QSerialPort::OneStop); break;
        case 1: serial.setStopBits(QSerialPort::TwoStop); break;
        default: break;
        }
        //设置流控制
        serial.setFlowControl(QSerialPort::NoFlowControl);
        //打开串口
        if(!serial.open(QIODevice::ReadWrite))
        {
            QMessageBox::about(NULL, "提示", "无法打开串口！");
           return;
        }
        //下拉菜单控件失能
        ui.portNameBox->setEnabled(false);
        ui.baudrateBox->setEnabled(false);
        ui.dataBitsBox->setEnabled(false);
        ui.ParityBox->setEnabled(false);
        ui.stopBitsBox->setEnabled(false);
        ui.openButton->setText(QString("关闭串口"));
        //发送按键使能
        ui.sendButton->setEnabled(true);
    }
    else
    {
        //关闭串口
        serial.close();
        //下拉菜单控件使能
        ui.portNameBox->setEnabled(true);
        ui.baudrateBox->setEnabled(true);
        ui.dataBitsBox->setEnabled(true);
        ui.ParityBox->setEnabled(true);
        ui.stopBitsBox->setEnabled(true);
        ui.openButton->setText(QString("打开串口"));
        //发送按键失能
        ui.sendButton->setEnabled(false);
    }
}

void MainWindow::on_sendButton_clicked()
{
    //获取界面上的数据并转换成utf8格式的字节流
    QByteArray data = ui.sendTextEdit->toPlainText().toUtf8();
    //serial.write(data);
    serial.write(QString2Hex(data));
}

void MainWindow::on_clearButton_clicked()
{
    ui.recvTextEdit->clear();
}

void MainWindow::agvPose_pushbtn_click()
{
    agv_pose_model=true;
    //slot_agvPose_image();
}

void MainWindow::agvPath_pushbtn_click()
{
    agv_pose_model=false;
    //slot_agvPose_image();
}

//AGV轨迹显示
void MainWindow::slot_agvPose_image()
{
    //int w = 620, h = 360, y = 50;
    //白图黑线
    //Mat agv_image(h, w, CV_8UC3, Scalar(255, 255, 255));
    QImage Qtemp;
    //AGV的像素坐标
    int agv_image_x=(int)(agv_pose_x*12.0)+305;
    int agv_image_y=-(int)(agv_pose_y*12.0)+180;
    //AGV指向
    int agv_line_x=agv_image_x+(int)(12*cos(agv_pose_theta));
    int agv_line_y=agv_image_y-(int)(12*sin(agv_pose_theta));

    /*
    if(!agv_image_start && !agv_pose_model)
    {
        agv_image_start=true;
        line(agv_image, Point(3, 3), Point(agv_image.cols - 3, 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(agv_image.cols - 3, 3), Point(agv_image.cols - 3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(agv_image.cols - 3, agv_image.rows - 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(3, 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
    }
    */
    if(agv_pose_model)
    {
        agv_path_fast_start=false;
        //清空背景色
        Vec3b color;  //设置的背景色
        color[0] = 255;//rng.uniform(0, 255);
        color[1] = 255;// rng.uniform(0, 255);
        color[2] = 255;

        for (size_t row=0; row<360; row++)
        {
            for (size_t col=0;col<610;col++)
            {
                //设置原图像中某点的BGR颜色值
                agv_image.at<Vec3b>(row, col) = Vec3b(color(0), color(1), color(2));
            }
        }
        line(agv_image, Point(3, 3), Point(agv_image.cols - 3, 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(agv_image.cols - 3, 3), Point(agv_image.cols - 3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(agv_image.cols - 3, agv_image.rows - 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
        line(agv_image, Point(3, 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);

        circle(agv_image, Point(agv_image_x, agv_image_y), 5, Scalar(255,0,0));
        line(agv_image, Point(agv_image_x, agv_image_y), Point(agv_line_x, agv_line_y), Scalar(255, 0, 0));
    }
    else
    {
        if(!agv_path_fast_start)
        {
            agv_path_fast_start=true;
            //清空背景色
            Vec3b color;  //设置的背景色
            color[0] = 255;//rng.uniform(0, 255);
            color[1] = 255;// rng.uniform(0, 255);
            color[2] = 255;

            for (size_t row=0; row<360; row++)
            {
                for (size_t col=0;col<610;col++)
                {
                    //设置原图像中某点的BGR颜色值
                    agv_image.at<Vec3b>(row, col) = Vec3b(color(0), color(1), color(2));
                }
            }
            line(agv_image, Point(3, 3), Point(agv_image.cols - 3, 3), Scalar(0, 0, 0), 2);
            line(agv_image, Point(agv_image.cols - 3, 3), Point(agv_image.cols - 3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
            line(agv_image, Point(agv_image.cols - 3, agv_image.rows - 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
            line(agv_image, Point(3, 3), Point(3, agv_image.rows - 3), Scalar(0, 0, 0), 2);
        }
        line(agv_image, Point(agv_image_x, agv_image_y), Point(agv_last_x, agv_last_y), Scalar(0, 0, 255));
    }
    //上一个AGV点
    agv_last_x=agv_image_x;
    agv_last_y=agv_image_y;
    agv_last_theta=agv_pose_theta;

    Qtemp = QImage((const unsigned char*)(agv_image.data), agv_image.cols, agv_image.rows, agv_image.step, QImage::Format_RGB888);
    ui.label_agv_image->setPixmap(QPixmap::fromImage(Qtemp));
    //ui.label_agv_image>resize(Qtemp.size());
    ui.label_agv_image->show();
}

//realsense相机图像显示
void MainWindow::slot_update_image(QImage img)
{
    QPixmap pixmap=QPixmap::fromImage(img);
    pixmap = pixmap.scaled(320, 480, Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    ui.label_image->setPixmap(pixmap);   //显示图像
}

void MainWindow::slot_sub_image()
{
    if(ui.lineEdit_image_topic->text().toStdString()=="")
    {
        qnode.sub_image();
    }
    else
    {
        qnode.sub_image(ui.lineEdit_image_topic->text());
    }
}

void MainWindow::quit_ui()
{
     QApplication::exit();
}

//3D扫描点云显示
void MainWindow::pcl3D_show()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3D(new pcl::PointCloud<pcl::PointXYZ>);
    qnode.pcl_read(cloud_3D);
    cloud_3D_save=*cloud_3D;
    if(pointcloud_btn_flag)
    {
        pointcloud_btn_flag=false;
        viewer->removePointCloud("cloud");
    }
    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1);
    if(point3D_first)
    {
        point3D_first=false;
        cout << "扫描到3D点的数量为：" <<cloud_3D->points.size() << endl;
        viewer->addPointCloud(cloud_3D,"cloud_3D");
    }
    else
    {
        viewer->updatePointCloud(cloud_3D,"cloud_3D");
    }

    while (!viewer->wasStopped())
    {
        //cout << "点云显示开始！"<< endl;
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    //cout << "点云显示结束！"<< endl;
}
//点云显示
void MainWindow::slot_point_open()
{
    /*
    //ui.textEdit->setText("hello world!");
    Mat img = imread("/home/binghun/serialPort/src/realsense_camera/data/pict_rgb0.jpg");
    Mat img2;
    //imshow("image", img);
    waitKey(0);

    double scale = 0.5;
    //int shrinkType;
    //int resizeTypeValue = 0;
    //shrinkType = typeDef(resizeTypeValue);
    Size dsize = Size(img.cols*scale, img.rows*scale);
    img2 = Mat(dsize, CV_32S);
    cv::resize(img,img2,dsize, 0, 0, INTER_LANCZOS4);

    Mat temp;
    cvtColor(img2, temp, CV_BGR2RGB);
    QImage Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    ui.label_8->setPixmap(QPixmap::fromImage(Qtemp));
    //ui.label_8->resize(Qtemp.size());
    ui.label_8->show();
    */

    bool rbg_point_flag=ui.checkBox_RGB->isChecked();
    if(rbg_point_flag)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //第一种读入方法较多场合如此
        if(ui.lineEdit_openpath->text().toStdString()=="")
        {
            // /home/binghun/pcd/1641976423_1.pcd
            // /home/binghun/pcd/linux3.pcd
            char strfilepath[256] = "/home/binghun/serialPort/src/realsense_camera/data/pointcloud.pcd";
            if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud))
            {
               cout << "error input!" << endl;
               return;
            }
        }
        else
        {
            string point_path=ui.lineEdit_openpath->text().toStdString();
            if (-1 == pcl::io::loadPCDFile(point_path, *cloud))
            {
               cout << "error input!" << endl;
               return;
            }
        }

        cloud_3DRGB_save=*cloud;

        //viewer->setBackgroundColor(0.1,0.1,0);
        viewer->setBackgroundColor((double)background_red/255.0,(double)background_green/255.0,(double)background_blue/255.0);
        viewer->addCoordinateSystem(1);

        if(!point3D_first)
        {
            viewer->removePointCloud("cloud_3D");
            viewer->addPointCloud(cloud,"cloud");
        }
        else if(pointcloud_btn_flag)
        {
            viewer->updatePointCloud(cloud,"cloud");
        }
        else
        {
            viewer->addPointCloud(cloud,"cloud");
        }

        //viewer->initCameraParameters();	 //初始化相机参数，让用户在默认的角度下观察点云

        std::cout << "read completed" << std::endl;
        cout << cloud->points.size() << endl;

    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        //第一种读入方法较多场合如此
        if(ui.lineEdit_openpath->text().toStdString()=="")
        {
            char strfilepath[256] = "/home/binghun/pcd/linux3.pcd";
            if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud2))
            {
               cout << "error input!" << endl;
               return;
            }
        }
        else
        {
            string point_path=ui.lineEdit_openpath->text().toStdString();
            if (-1 == pcl::io::loadPCDFile(point_path, *cloud2))
            {
               cout << "error input!" << endl;
               return;
            }
        }
        cloud_3D_save=*cloud2;

        //viewer->setBackgroundColor(0.1,0.1,0);
        viewer->setBackgroundColor((double)background_red/255.0,(double)background_green/255.0,(double)background_blue/255.0);
        viewer->addCoordinateSystem(1);

        if(!point3D_first)
        {
            viewer->removePointCloud("cloud_3D");
            viewer->addPointCloud(cloud2,"cloud");
        }
        else if(pointcloud_btn_flag)
        {
            viewer->updatePointCloud(cloud2,"cloud");
        }
        else
        {
            viewer->addPointCloud(cloud2,"cloud");
        }

        //viewer->initCameraParameters();	 //初始化相机参数，让用户在默认的角度下观察点云

        std::cout << "read completed" << std::endl;
        cout << cloud2->points.size() << endl;
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    pointcloud_btn_flag=true;
}

//点云保存
void MainWindow::slot_point_save()
{
    bool rbg_point_flag=ui.checkBox_RGB->isChecked();
    string point_path;
    if(ui.lineEdit_savepath->text().toStdString()=="")
    {
        point_path= "/home/binghun/pcd/sample.pcd";
    }
    else
    {
        point_path=ui.lineEdit_openpath->text().toStdString();
    }

    if(rbg_point_flag)
    {
        pcl::io::savePCDFile(point_path,cloud_3DRGB_save);
    }
    else
    {
        pcl::io::savePCDFile(point_path,cloud_3D_save);
    }
}

/*
void MainWindow::slot_quick_cmd_clicked()
{
    laser_cmd=new QProcess;
    laser_cmd->start("bash");   //可以运行命令行
    //toPlainText()获取文本框的文字
    //fromStdString转换为utf-8的QString，而fromLocal8Bit转化为的是GBK的QString
    laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+'\n');
    //有错误信号发送时，跳到slot_quick_output()
    connect(laser_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_output()));
    connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));

}

void MainWindow::slot_quick_output()
{
    //<font color=\"#FF0000\">  255 00 00 RGB,红色
    ui.textEdit_quick_output->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font>");
    //正常输出，白色qDebug()<<btn->text();
    ui.textEdit_quick_output->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font>");
}
*/

void MainWindow::slot_update_power(float value)
{
    //QString::number是将数数字（整数、浮点数、有符号、无符号等）转换为QString类型
    //第一个参数：待转换数字
    //第二个参数（整型）：转换进制
    //第二个参数（浮点数）：浮点数格式
    //第三个参数（浮点数）：保留小数位数
    //QString QString::mid(int position, int n = -1) const  返回一个从position开始，长度为n的QString 类型的子串
    //当从position开始的子串长度不够n或n为-1（缺省时的默认值也为-1），函数返回从position开始到结尾的子串
    ui.label_power_value->setText(QString::number(value).mid(0,5)+"V");
    double n=(value-10.5)/(12.5-10.5);   //最低电压10.5V，最高电压12.5V
    int val=n*100;
    ui.progressBar->setValue(val);
}

void MainWindow::slot_update_dashboard(float x,float w,float pose_x,float pose_y,float pose_theta)
{
    agv_pose_x=pose_x;
    agv_pose_y=pose_y;
    agv_pose_theta=pose_theta;

    //显示AGV位置
    agv_pLCD_x->display(pose_x);
    agv_pLCD_y->display(pose_y);
    agv_pLCD_theta->display(pose_theta);

    ui.label_dir_x->setText(x>0?"正向":"反向");
    ui.label_dir_y->setText(w>0?"正向":"反向");
    if(x<0) x=-x;
    if(w<0) w=-w;
    if(x>1) x=1;
    if(w>3) w=3;

    speed_x_dashBoard->setValue(x*100);  //仪表盘的默认数值
    speed_y_dashBoard->setValue(w);

    slot_agvPose_image();
}

void MainWindow::radar_pushbtn_click()
{
    //sender获取哪个对象的信号
    QPushButton* btn=qobject_cast<QPushButton*> (sender());
    //qDebug()<<btn->text();
    //qDebug()<<'k';
    //QString ;
    std::string radat_control=btn->text().toStdString();

    int radar_int=-1;
    if(radat_control=="打开激光雷达")
    {
       radar_int=1;
    }
    else if(radat_control=="关闭激光雷达")
    {
        radar_int=0;
    }
    qnode.set_ladar(radar_int);
}

void MainWindow::motor_pushbtn_click()
{
    //sender获取哪个对象的信号
    QPushButton* btn=qobject_cast<QPushButton*> (sender());
    //qDebug()<<btn->text();
    //qDebug()<<'k';
    //QString ;
    std::string motor_control=btn->text().toStdString();

    bool is_continous=ui.checkBox_continous->isChecked();
    bool is_tiny=ui.checkBox_tiny->isChecked();

    int motor_int=-1;
    int speed_level = -1;

    switch(ui.speedBox->currentIndex())
    {
        case 0: speed_level=0; break;
        case 1: speed_level=1; break;
        case 2: speed_level=2; break;
        case 3: speed_level=3; break;
        case 4: speed_level=4; break;
        default: break;
    }

    if(motor_control=="初始化")
    {
       motor_int=1;
       ui.motor_forward_btn->setEnabled(true);
       ui.motor_reverses_btn->setEnabled(true);
       ui.motorpos_init_btn->setEnabled(true);
    }
    else if(motor_control=="电机脱机")
    {
        motor_int=3;
    }
    else if(motor_control=="电机抱死")
    {
        motor_int=2;
    }
    else if(motor_control=="电机正转")
    {
        if(is_tiny)
        {
            motor_int=12;
        }
        else
        {
            motor_int=6;
        }
    }
    else if(motor_control=="电机反转")
    {
        if(is_tiny)
        {
            motor_int=8;
        }
        else if(is_continous)
        {
            motor_int=9;
        }
        else
        {
            motor_int=7;
        }

    }
    else if(motor_control=="电机急停")
    {
        motor_int=4;
        ui.motor_offline_btn->setEnabled(true);
        ui.motorpos_read_btn->setEnabled(true);
        ui.motor_init_btn->setEnabled(true);
        ui.motor_lock_btn->setEnabled(true);
    }
    else if(motor_control=="位置初始化")
    {
        ui.lcdNumber_motor->setEnabled(true);
        ui.motorpos_read_btn->setEnabled(true);
        motor_int=10;
    }
    else if(motor_control=="位置读取")
    {
        motor_int=13;
    }
    qnode.set_motor(motor_int, speed_level);
}

void MainWindow::imu_pushbtn_click()
{
    QPushButton* btn=qobject_cast<QPushButton*> (sender());
    std::string radat_control=btn->text().toStdString();

    int imu_int=-1;
    if(radat_control=="初始化IMU")
    {
        ui.imu_open_btn->setEnabled(true);
        ui.imu_close_btn->setEnabled(true);
        imu_int=0;
    }
    else if(radat_control=="打开IMU")
    {
        imu_int=1;
    }
    else if(radat_control=="关闭IMU")
    {
        imu_int=2;
    }
    qnode.set_imu(imu_int);
}

void MainWindow::point_pushbtn_click()
{
    //sender获取哪个对象的信号
    QPushButton* btn=qobject_cast<QPushButton*> (sender());
    //qDebug()<<btn->text();
    //qDebug()<<'k';
    //QString ;
    std::string radat_control=btn->text().toStdString();

    bool cycle_flag=ui.checkBox_pointIsC->isChecked();
    bool scanMethod_flag=ui.scanMethod_checkBox->isChecked();
    int start_int=-1;
    int speed_level = -1;

    switch(ui.speedBox->currentIndex())
    {
        case 0: speed_level=0; break;
        case 1: speed_level=1; break;
        case 2: speed_level=2; break;
        case 3: speed_level=3; break;
        case 4: speed_level=4; break;
        default: break;
    }
    if(radat_control=="3D点云采集")
    {
        ui.motor_forward_btn->setEnabled(false);
        ui.motor_reverses_btn->setEnabled(false);
        ui.motorpos_init_btn->setEnabled(false);
        ui.motor_init_btn->setEnabled(false);
        ui.motor_lock_btn->setEnabled(false);
        ui.motor_offline_btn->setEnabled(false);
        ui.motorpos_read_btn->setEnabled(false);
       if(scanMethod_flag)
       {
           start_int=3;
       }
       else if(cycle_flag)
       {
           start_int=2;
       }
       else
       {
           start_int=1;
       }
    }
    else if(radat_control=="后停止采集")
    {
        start_int=0;
        ui.motor_init_btn->setEnabled(true);
        ui.motor_lock_btn->setEnabled(true);
    }
    qnode.set_start(start_int,speed_level);
}

void MainWindow::motorAxis_get(int motorAxis)
{
    double motor_angle;
    motor_angle=motorAxis*180.0/2006;
    motor_pLCD->display(motor_angle);
}

void MainWindow::slot_pushbtn_click()
{
    //sender获取哪个对象的信号
    QPushButton* btn=qobject_cast<QPushButton*> (sender());
    //qDebug()<<btn->text();
    //qDebug()<<'k';
    char k=btn->text().toStdString()[0];

    bool is_all=ui.checkBox_is_all->isChecked();
    float linear=ui.label_linear->text().toFloat()*0.01;   //界面为里面，转化为米
    float angular=ui.label_raw->text().toFloat()*0.01;

    switch(k){
    case 'i':
        qnode.set_cmd_vel(is_all?'I':'i',linear,angular);
        break;
    case 'u':
        qnode.set_cmd_vel(is_all?'U':'u',linear,angular);
        break;
    case 'o':
        qnode.set_cmd_vel(is_all?'O':'o',linear,angular);
        break;
    case 'j':
        qnode.set_cmd_vel(is_all?'J':'j',linear,angular);
        break;
    case 'k':
        qnode.set_cmd_vel(is_all?'K':'k',linear,angular);
        break;
    case 'l':
        qnode.set_cmd_vel(is_all?'L':'l',linear,angular);
        break;
    case 'm':
        qnode.set_cmd_vel(is_all?'M':'m',linear,angular);
        break;
    case ',':
        qnode.set_cmd_vel(is_all?'>':',',linear,angular);
        break;
    case '.':
        qnode.set_cmd_vel(is_all?'I':'.',linear,angular);
        break;
    }
}

void MainWindow::slot_linear_value_change(int value)
{
    ui.label_linear->setText(QString::number(value));
}

void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//按钮点击事件
void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {    //使用环境变量，不使用的话使用master和ip进行连接
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_hmi");
    restoreGeometry(settings.value("geometry").toByteArray());   //窗口的大小
    restoreState(settings.value("windowState").toByteArray());   //窗口的状态
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();   //本地IP
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_hmi");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_hmi

