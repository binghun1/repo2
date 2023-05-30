/**
 * @file /include/robot_hmi/main_window.hpp
 *
 * @brief Qt based gui for robot_hmi.
 *
 * @date November 2010
 **/
#ifndef robot_hmi_MAIN_WINDOW_H
#define robot_hmi_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
//#include "ui_main_window.h"
//#include "qnode.hpp"
//#include "CCtrlDashBoard.h"
#include <ros/ros.h>

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.h"
#include <QImage>
#include <QProcess>

#include <vtkRenderWindow.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include "vtkConeSource.h"
#include "vtkConeSource.h"
#include "vtkCommand.h"
#include "vtkCamera.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkTransform.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkDataSet.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkArcSource.h>
#include <vtkAppendPolyData.h>
#include <vtkScalarBarActor.h>
#include <vtkKdTree.h>
#include <vtkLODActor.h>
#include <vtkMath.h>
#include <vtkWindowToImageFilter.h>
#include <vtkBMPWriter.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>   //pcd //读写类相关的头文件。
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

//串口
#include <QSerialPort>        //提供访问串口的功能
#include <QSerialPortInfo>    //提供系统中存在的串口的信息

//时间
#include <QLCDNumber>
#include <QTimer>
#include <QDateTime>
#include "ros/time.h"

#include <sensor_msgs/Imu.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

using namespace cv;
namespace robot_hmi {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linear_value_change(int);
    void slot_raw_value_change(int);
    void slot_pushbtn_click();
    void slot_update_dashboard(float,float,float,float,float);
    void slot_update_power(float);
    //void slot_quick_cmd_clicked();
    //void slot_quick_output();
    void quit_ui();
    void slot_update_image(QImage);
    void slot_sub_image();

    //AGV轨迹
    void slot_agvPose_image();

    //串口
    void serialPort_readyRead();
    void on_searchButton_clicked();
    void on_openButton_clicked();
    void on_sendButton_clicked();
    void on_clearButton_clicked();

    //雷达
    void radar_pushbtn_click();
    //电机
    void motor_pushbtn_click();
    void motorAxis_get(int);
    //IMU
    void imu_pushbtn_click();
    void imuData_get(float,float,float,float,float,float,float,float,float);
    //点云
    void slot_point_open();
    void slot_point_save();
    void point_pushbtn_click();
    void pcl3D_show();
    void slot_red_value_change(int);
    void slot_green_value_change(int);
    void slot_blue_value_change(int);
    void slot_size_value_change(int);

    //轨迹显示
    void agvPose_pushbtn_click();
    void agvPath_pushbtn_click();

    //时间
    void onTimeOut();

    QByteArray QString2Hex(QString hexStr);
    char ConvertHexChar(char c);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    CCtrlDashBoard* speed_x_dashBoard;
    CCtrlDashBoard* speed_y_dashBoard;
    QProcess *laser_cmd;
    QSerialPort serial;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    QLCDNumber *m_pLCD;  //时钟
    QLCDNumber *m_pLCD_ros;
    QLCDNumber *motor_pLCD;
    QLCDNumber *imu_pLCD1;
    QLCDNumber *imu_pLCD2;
    QLCDNumber *imu_pLCD3;
    QLCDNumber *imu_pLCD4;
    QLCDNumber *imu_pLCD5;
    QLCDNumber *imu_pLCD6;
    QLCDNumber *imu_pLCD7;
    QLCDNumber *imu_pLCD8;
    QLCDNumber *imu_pLCD9;
    QLCDNumber *agv_pLCD_x;
    QLCDNumber *agv_pLCD_y;
    QLCDNumber *agv_pLCD_theta;

    bool point3D_first=true;         //第一次接收到雷达3D点云
    bool pointcloud_btn_flag=false;  //按钮接收点云
    int background_red=0;
    int background_green=0;
    int background_blue=0;
    int point_size=1;
    //Mat agvpose_Img;
    Mat agv_image;  //AGV的运动轨迹
    bool agv_image_start=false;
    bool agv_pose_model=true;
    bool agv_path_fast_start=false;

    //AGV的位置
    float agv_pose_x=0;
    float agv_pose_y=0;
    float agv_pose_theta=0;
    //上一时刻像素
    float agv_last_x=305;
    float agv_last_y=180;
    float agv_last_theta=0;

    pcl::PointCloud<pcl::PointXYZ> cloud_3D_save;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_3DRGB_save;


};

}  // namespace robot_hmi

#endif // robot_hmi_MAIN_WINDOW_H
