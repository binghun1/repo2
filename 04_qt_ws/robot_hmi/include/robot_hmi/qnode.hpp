/**
 * @file /include/robot_hmi/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_hmi_QNODE_HPP_
#define robot_hmi_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>  //图像编码格式

#include <QImage>
#include <sensor_msgs/Imu.h>
#include "ros/time.h"
#include <tf/tf.h>

//点云
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void set_cmd_vel(char k,float linear,float angular);
    //void sub_image(QString topic_name);
    void sub_image(QString topic_name="/camera/color/image_raw");

    //雷达
    void set_ladar(int k);
    //电机
    void set_motor(int k, int speed_level);
    void getTime(double &time);
    void set_start(int k, int speed_level);
    void set_imu(int k);
    //点云
    void pcl_read(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    geometry_msgs::Twist twist;
    bool ros_init_flag=false;
    pcl::PointCloud<pcl::PointXYZ> cloud_3D;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:  //信号
	void loggingUpdated();
    void rosShutdown();
    void speed_vel(float,float,float,float,float);
    void power_vel(float);
    void image_vel(QImage);
    void motorAxis_vel(int);
    void imu_vel(float,float,float,float,float,float,float,float,float);
    void pcl_vel();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher cmd_vel_pub;
    ros::Publisher radar_control_pub;
    ros::Publisher motor_control_pub;
    ros::Publisher speed_control_pub;
    ros::Publisher start_control_pub;
    ros::Publisher imu_control_pub;
    QStringListModel logging_model;
    ros::Subscriber chatter_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber power_sub;
    ros::Subscriber motorAxis_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber pcl_sub;
    ros::Subscriber new_cycle_sub;
    //image_transport::Subscriber image_sub;
    ros::Subscriber image_sub;
    void motorAxis_callback(const std_msgs::Int16 &msg);
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
    void power_callback(const std_msgs::Float32 &msg);
    void chattter_callback(const std_msgs::String &msg);
    void odom_callback(const nav_msgs::Odometry &msg);
    void imu_callback(const sensor_msgs::Imu &msg);
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void newcycle_callback(const std_msgs::Int8 &msg);
    QImage Mat2QImage(cv::Mat const& src);

    //变量
    bool pcl_read_flag=true;
    int pre_speed=-1;
};

}  // namespace robot_hmi

#endif /* robot_hmi_QNODE_HPP_ */
