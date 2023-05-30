/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <QMessageBox>
#include <iostream>
#include <QDebug>
#include "../include/robot_hmi/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

//using namespace pcl;
//using namespace pcl::io;
using namespace std;

namespace robot_hmi {
//using namespace Qt;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::newcycle_callback(const std_msgs::Int8 &msg)
{
    pcl_read_flag=true;
}

void QNode::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    if(pcl_read_flag)
    {
        pcl_read_flag=false;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3D(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_msg, *cloud);
        pcl::fromPCLPointCloud2 (*cloud, cloud_3D);
        cout <<"扫描的3D点云数量为："<< cloud_3D.points.size() << endl;
        emit pcl_vel();
    }
}

void QNode::pcl_read(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    *cloud=cloud_3D;
}

void QNode::chattter_callback(const std_msgs::String &msg)
{
    log(Info,"I recive "+msg.data);
}

void QNode::odom_callback(const nav_msgs::Odometry &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    emit speed_vel(msg.twist.twist.linear.x,msg.twist.twist.angular.z,msg.pose.pose.position.x,msg.pose.pose.position.y,yaw);  //发出信号
}

void QNode::imu_callback(const sensor_msgs::Imu &msg)
{

    float imu_data[9];
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(msg.orientation,RQ2);
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
    imu_data[0]=roll*180/M_PI;
    imu_data[1]=pitch*180/M_PI;
    imu_data[2]=yaw*180/M_PI;
    imu_data[3]=msg.angular_velocity.x;
    imu_data[4]=msg.angular_velocity.y;
    imu_data[5]=msg.angular_velocity.z;
    imu_data[6]=msg.linear_acceleration.x;
    imu_data[7]=msg.linear_acceleration.y;
    imu_data[8]=msg.linear_acceleration.z;

    //log(Info,"I recive "+std::to_string(imu_data[2]));

    emit imu_vel(imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],imu_data[6],imu_data[7],imu_data[8]);
}

//环境变量
bool QNode::init() {
    ros::init(init_argc,init_argv,"robot_hmi");  //初始化节点
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    radar_control_pub = n.advertise<std_msgs::Int8>("/radar_send_control",1000);
    motor_control_pub = n.advertise<std_msgs::Int8>("/motor_control",1000);
    speed_control_pub = n.advertise<std_msgs::Int8>("/speed_control",1000);
    start_control_pub = n.advertise<std_msgs::Int8>("/start_control",1000);
    imu_control_pub = n.advertise<std_msgs::Int8>("/imu_control",1000);
    //chatter_sub=n.subscribe("chatter",100,&QNode::chattter_callback,this);
    odom_sub=n.subscribe("odometry",1000,&QNode::odom_callback,this);   //odometry
    power_sub=n.subscribe("power",1000,&QNode::power_callback,this);
    motorAxis_sub=n.subscribe("/ConsMotorAxis",1000,&QNode::motorAxis_callback,this);
    imu_sub=n.subscribe("/imu_data",1000,&QNode::imu_callback,this);
    pcl_sub=n.subscribe("/pcl_output",1000,&QNode::pcl_callback,this);
    new_cycle_sub = n.subscribe("/qt_new_cycle",1000,&QNode::newcycle_callback,this);
    start();   //进入run函数
	return true;
}

//IP地址连接
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_hmi");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    radar_control_pub = n.advertise<std_msgs::Int8>("/radar_send_control",1000);
    motor_control_pub = n.advertise<std_msgs::Int8>("/motor_control",1000);
    speed_control_pub = n.advertise<std_msgs::Int8>("/speed_control",1000);
    start_control_pub = n.advertise<std_msgs::Int8>("/start_control",1000);
    imu_control_pub = n.advertise<std_msgs::Int8>("/imu_control",1000);
    //chatter_sub=n.subscribe("chatter",100,&QNode::chattter_callback,this);
    odom_sub=n.subscribe("odometry",1000,&QNode::odom_callback,this);
    power_sub=n.subscribe("power",1000,&QNode::power_callback,this);
    motorAxis_sub=n.subscribe("/ConsMotorAxis",1000,&QNode::motorAxis_callback,this);
    imu_sub=n.subscribe("/imu_data",1000,&QNode::imu_callback,this);
    pcl_sub=n.subscribe("/pcl_output",1000,&QNode::pcl_callback,this);
    new_cycle_sub = n.subscribe("/qt_new_cycle",1000,&QNode::newcycle_callback,this);
	start();
	return true;
}

void QNode::motorAxis_callback(const std_msgs::Int16 &msg)
{
    emit motorAxis_vel(msg.data);
}

void QNode::getTime(double &time)
{
    time=ros::Time::now().toSec();
}

void QNode::sub_image(QString topic_name)
{
    log(Info,"I recive "+topic_name.toStdString());
    ros::NodeHandle n;
    //qDebug()<<'o';
    //image_transport::ImageTransport it_(n);
    //image_sub=it_.subscribe(topic_name.toStdString(),1000,&QNode::image_callback,this);
    image_sub=n.subscribe(topic_name.toStdString(),1000,&QNode::image_callback,this);
}

void QNode::power_callback(const std_msgs::Float32 &msg)
{
     emit power_vel(msg.data);
}

void QNode::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    //qDebug()<<'l';
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg, "bgr8");    //格式转换  "mono8"  msg->encoding
    //OPENCV转为QT图像
    QImage img=Mat2QImage(cv_ptr->image);
    emit image_vel(img);
}

QImage QNode::Mat2QImage(cv::Mat const& src)
{
  QImage dest(src.cols, src.rows, QImage::Format_RGB888);  //QImage::Format_RGB888   QImage::Format_ARGB32

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}

void QNode::set_imu(int k)
{
    std_msgs::Int8 imu_int;
    imu_int.data=-1;

    if(k==-1)
    {
        log(Error,"IMU参数错误！");
    }
    else if(k>=0 && k<=2)
    {
        imu_int.data=k;
        imu_control_pub.publish(imu_int);
    }
}

void QNode::set_motor(int k, int speed_level)
{
    std_msgs::Int8 motor_int;
    std_msgs::Int8 speed_int;
    motor_int.data=-1;

    if(k==-1)
    {
        log(Error,"电机参数错误！");
    }
    else if(k>=0 && k<=13)
    {
        motor_int.data=k;
        motor_control_pub.publish(motor_int);
    }
    if(speed_level==-1)
    {
        log(Error,"转速参数错误！");
    }
    //pre_speed != speed_level &&
    else if(speed_level>=0 && speed_level<=4)
    {
        speed_int.data=speed_level;
        pre_speed=speed_level;
        speed_control_pub.publish(speed_int);
    }
}

void QNode::set_ladar(int k)
{
    std_msgs::Int8 radar_int;
    radar_int.data=-1;
    switch(k)
    {
    case -1:
        log(Error,"雷达参数错误！");
        break;
    case 0:
        radar_int.data=0;
        break;
    case 1:
        radar_int.data=1;
        break;
    default:
        break;
    }
    radar_control_pub.publish(radar_int);
}

void QNode::set_start(int k, int speed_level)
{
    std_msgs::Int8 start_int;
    std_msgs::Int8 speed_int;
    start_int.data=-1;
    if(k==-1)
    {
        log(Error,"开始参数错误！");
    }
    else if(k>=0 && k<=3)
    {
        start_int.data=k;
        start_control_pub.publish(start_int);
    }
    if(speed_level==-1)
    {
        log(Error,"转速参数错误！");
    }
    else if(speed_level>=0 && speed_level<=4)
    {
        speed_int.data=speed_level;
        pre_speed=speed_level;
        speed_control_pub.publish(speed_int);
    }
}

void QNode::set_cmd_vel(char k,float linear,float angular)
{
    // Map for movement keys
    //定义map容器
    std::map<char, std::vector<float>> moveBindings
    {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'k', {0, 0, 0, 0}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'K', {0, 0, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}}
    };

    char key=k;  //容器的查询钥匙
    //方向数据
    int x=moveBindings[key][0];
    int y=moveBindings[key][1];
    int z=moveBindings[key][2];
    int th=moveBindings[key][3];


    twist.linear.x=x*linear;
    twist.linear.y=y*linear;
    twist.linear.z=z*linear;

    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=th*angular;
    //log(Info,std::string("I sent: "));
    qDebug()<<twist.linear.x;

    cmd_vel_pub.publish(twist);
    //qDebug()<<'l';
}

void QNode::run() {   //不影响主线程界面
    ros::Rate loop_rate(30);
    //int count = 0;
    ros_init_flag=true;
	while ( ros::ok() ) {

        //std_msgs::String msg;
        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();
        //chatter_publisher.publish(msg);
        //log(Info,std::string("I sent: ")+msg.data);
        ros::spinOnce();  //调用回调函数
		loop_rate.sleep();
        //++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace robot_hmi
