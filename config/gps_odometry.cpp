/* This package is to transform the UTM cordiates in rviz coordinates
 *              |Y                      X|
 * And with the relative position to the first
 *    From      |           To           |  reading
 *              |_____X            Y_____|
*******************************************************************************************/

#include <ros/ros.h>
#include <ros/param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>

std::string g_input_gps_topic_;
std::string g_input_compass_topic_;
std::string g_output_odom_topic_;
std::string g_input_reset_topic_;
float g_initial_x_ = 0.0;
float g_initial_y_ = 0.0;
float g_compass_rad_ = 0.0;

bool g_first_reading_ = true;
bool g_first_compass_ = true;

ros::Publisher g_odometry_pub_;
std::string vehicle_;
bool with_orientation_;
bool first_angle_ = true;
std::string local_frame_id_;
std::string link_frame_;

tf::TransformListener* listener_;

//
double angleLimiting(double angle)
{
  double output = atan2(sin(angle), cos(angle));
  return output;
}

//
void compassCallBack(const std_msgs::Float64ConstPtr& compass)
{
  // this statement is only to receive the first reading
  if (g_first_compass_) // g_reset || g_first_compass)
  {
    g_compass_rad_ = compass->data * M_PI / 180;
    g_first_compass_ = false;
  }
}

//
void gpsOrientationCalculation(nav_msgs::Odometry odom)
{
  // storing the previous values of the gps odometry for the angle calculation
  static double prev_local_x;
  static double prev_local_y;
  static double local_theta;
  // storing the first covariances values outputed by the gps module
  static Eigen::Matrix<double, 6, 6> initial_covariance;
  // buffers for storing the previous values of the standard deviations
  static double prev_std_x = 0;
  static double prev_std_y = 0;
  static double initial_std;
  double std_theta;

  double local_x;
  double local_y;
  double delta_theta;
  double std_delta_theta;

  double std_x;
  double std_y;

  Eigen::Matrix<double, 6, 6> new_gps_covariance;

  Eigen::Matrix<double, 6, 6> gps_covariance;

  local_x = odom.pose.pose.position.x;
  local_y = odom.pose.pose.position.y;

  // filling the gps_covariance matrix from the odom message
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      gps_covariance(i, j) = odom.pose.covariance[(i * 6) + j];
    }
  }

  // if this is the first iteration of the function, store the output and the
  // initial covariance and the initial standard deviation
  if (first_angle_)
  {
    prev_local_x = local_x;
    prev_local_y = local_y;
    local_theta = 0;
    initial_covariance = gps_covariance;
    initial_std = atan2(gps_covariance(1, 1), gps_covariance(0, 0));
    first_angle_ = false;
  }

  // limiting the changes in the y and x to 0.3 in order to avoid calculating
  // the angle from fake output changes. (however, still there will be
  // unavoidable sudden changes in the values of the angle due to the stochastic
  // errors in the gps)
  if (abs(local_y - prev_local_y) < 0.3)
  {
    local_y = prev_local_y;
  }

  if (abs(local_x - prev_local_x) < 0.3)
  {
    local_x = prev_local_x;
  }

  // avoiding infinities when calculating the slope of the vehicle using the
  // straight line equation

  if ((local_x - prev_local_x) == 0 && (local_y - prev_local_y) != 0 &&
      local_theta > 0)
  {
    local_theta = M_PI / 2;
  }
  else if ((local_x - prev_local_x) == 0 && (local_y - prev_local_y) != 0 &&
           local_theta < 0)
  {
    local_theta = -M_PI / 2;
  }
  else if (abs(local_x - prev_local_x) < 0.5 ||
           abs(local_y - prev_local_y) == 0.5)
  {
    local_theta = local_theta;
  }
  else
  {
    local_theta = atan2(local_y - prev_local_y, local_x - prev_local_x);
  }
  // calculating the quaternion values of the calculated angle
  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(angleLimiting(local_theta));

  // putting the angle in the message to be published
  odom.pose.pose.orientation = odom_quat;

  // calculating the angle covariance from the covariance of the x and y
  // position by direct propagation of the error
  std_x = sqrt(gps_covariance(0, 0));
  std_y = sqrt(gps_covariance(1, 1));
  std_delta_theta = atan2((std_y - prev_std_y), (std_x - prev_std_x));
  std_theta = initial_std / 2 + std_delta_theta;

  // add the value the angle to the gps covariance
  gps_covariance(5, 5) = pow(std_theta, 2);

  odom.pose.covariance[35] = gps_covariance(5, 5);

  // updating the previous values of the gps
  prev_local_x = local_x;
  prev_local_y = local_y;
  prev_std_x = std_x;
  prev_std_y = std_y;

  g_odometry_pub_.publish(odom);
}

//
void gpsCallBack(const nav_msgs::OdometryConstPtr& gps_input_odom_msg)
{
  nav_msgs::Odometry gps_output_odom_msg;

  float x_pos = 0.0;
  float y_pos = 0.0;
  float x_pos_rot = 0.0;
  float y_pos_rot = 0.0;

  // First reading
  if (g_first_compass_)
  {
    // Waiting the first compass reading
  }
  else if (g_first_reading_ && !g_first_compass_)
  {
    g_initial_x_ = gps_input_odom_msg->pose.pose.position.x;
    g_initial_y_ = gps_input_odom_msg->pose.pose.position.y;
    g_first_reading_ = false;
  }
  else
  {
    // Substract the first reading to all positions
    x_pos = gps_input_odom_msg->pose.pose.position.x - g_initial_x_;
    y_pos = gps_input_odom_msg->pose.pose.position.y - g_initial_y_;

    // Rotation matrix to ego move from compass
    x_pos_rot = x_pos * cos(g_compass_rad_) - y_pos * sin(g_compass_rad_);
    y_pos_rot = x_pos * sin(g_compass_rad_) + y_pos * cos(g_compass_rad_);

    // change axes
    gps_output_odom_msg.pose.pose.position.x = y_pos_rot;
    gps_output_odom_msg.pose.pose.position.y = -x_pos_rot;

    //        cout << "x_pos: " << x_pos << " x_pos_rot: " << x_pos_rot << "
    //        x_pos_change: " << gps_output_odom_msg.pose.pose.position.x
    //        <<endl;
    //        cout << "y_pos: " << y_pos << " y_pos_rot: " << y_pos_rot << "
    //        y_pos_change: " << gps_output_odom_msg.pose.pose.position.y
    //        <<endl;

    gps_output_odom_msg.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(0);
    gps_output_odom_msg.pose.covariance = gps_input_odom_msg->pose.covariance;

    // wheel_odometry messages
    gps_output_odom_msg.header.seq = gps_input_odom_msg->header.seq;
    gps_output_odom_msg.header.stamp = gps_input_odom_msg->header.stamp;
    gps_output_odom_msg.header.frame_id =
        vehicle_ + "/" + gps_input_odom_msg->header.frame_id;

    tf::StampedTransform trans_form, initial_transform, odometry_transfrom;
    try
    {
      listener_->lookupTransform(
          vehicle_ + "/" + gps_input_odom_msg->header.frame_id,
          vehicle_ + "/" + link_frame_, ros::Time(0), initial_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    odometry_transfrom.setRotation(
        tf::Quaternion(gps_output_odom_msg.pose.pose.orientation.x,
                       gps_output_odom_msg.pose.pose.orientation.y,
                       gps_output_odom_msg.pose.pose.orientation.z,
                       gps_output_odom_msg.pose.pose.orientation.w));
    odometry_transfrom.setOrigin(
        tf::Vector3(gps_output_odom_msg.pose.pose.position.x,
                    gps_output_odom_msg.pose.pose.position.y,
                    gps_output_odom_msg.pose.pose.position.z));
    trans_form.setOrigin(tf::Vector3(0, 0, 0));
    trans_form.setRotation(tf::Quaternion(0, 0, 0, 1));
    trans_form.setOrigin(initial_transform.inverse().getOrigin() +
                         odometry_transfrom.getOrigin() +
                         odometry_transfrom.getBasis() *
                             initial_transform.getOrigin());
    trans_form.setRotation(odometry_transfrom.getRotation());

    gps_output_odom_msg.pose.pose.position.x = trans_form.getOrigin().getX();
    gps_output_odom_msg.pose.pose.position.y = trans_form.getOrigin().getY();
    gps_output_odom_msg.pose.pose.position.z = trans_form.getOrigin().getZ();
    gps_output_odom_msg.pose.pose.orientation.x =
        trans_form.getRotation().getX();
    gps_output_odom_msg.pose.pose.orientation.y =
        trans_form.getRotation().getY();
    gps_output_odom_msg.pose.pose.orientation.z =
        trans_form.getRotation().getZ();
    gps_output_odom_msg.pose.pose.orientation.w =
        trans_form.getRotation().getW();
    gps_output_odom_msg.header.frame_id = vehicle_ + "/" + local_frame_id_;
    gps_output_odom_msg.child_frame_id = vehicle_ + "/" + link_frame_;

    // calculate the angle from the gps readings
    if (with_orientation_)
      gpsOrientationCalculation(gps_output_odom_msg);
    else // if the user specifies not to calculate the angle then direclty
         // publish
      g_odometry_pub_.publish(gps_output_odom_msg);
  }
}

//
void resetCallBack(const std_msgs::EmptyConstPtr& empty)
{
  g_first_compass_ = true;
  g_first_reading_ = true;
  first_angle_ = true;
}

//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_odometry");
  ros::NodeHandle nh, nh_private("~");

  // public parameters
  if (!nh.getParam("vehicle", vehicle_) ||
      !nh.getParam("local_frame_id", local_frame_id_) ||
      !nh.getParam("link_frame", link_frame_))
  {
    std::string error_message =
        ros::this_node::getName() +
        ": failed to load global parameters, they are set to defaults!";
    ROS_ERROR("%s", error_message.c_str());
    vehicle_ = "";
    local_frame_id_ = "odom";
    link_frame_ = "base_link";
  }

  // private parameters
  if (!nh_private.getParam("input_gps_topic", g_input_gps_topic_) ||
      !nh_private.getParam("input_compass_topic", g_input_compass_topic_) ||
      !nh_private.getParam("reset_topic", g_input_reset_topic_) ||
      !nh_private.getParam("output_odometry_topic", g_output_odom_topic_) ||
      !nh_private.getParam("with_orientation", with_orientation_))
  {
    std::string error_message =
        ros::this_node::getName() +
        ": failed to load local parameters, please check the path!";
    ROS_ERROR("%s", error_message.c_str());
    ros::shutdown();
  }

  // Time sync gps + compass
  ros::Subscriber gps_sub =
      nh.subscribe(g_input_gps_topic_.c_str(), 1, gpsCallBack);
  ros::Subscriber g_compass_sub =
      nh.subscribe(g_input_compass_topic_.c_str(), 1, compassCallBack);
  ros::Subscriber reset_sub =
      nh.subscribe(g_input_reset_topic_.c_str(), 1, resetCallBack);

  // Global publisher inicialize
  g_odometry_pub_ =
      nh.advertise<nav_msgs::Odometry>(g_output_odom_topic_.c_str(), 1);

  listener_ = new tf::TransformListener();

  ros::spin();

  return 0;
}
