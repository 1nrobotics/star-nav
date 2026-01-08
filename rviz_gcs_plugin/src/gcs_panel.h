#ifndef GCS_PANEL_H
#define GCS_PANEL_H

#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <nav_msgs/Odometry.h>

namespace rviz_gcs_plugin
{

class GCSPanel : public rviz::Panel
{
Q_OBJECT
public:
  GCSPanel(QWidget* parent = 0);
  virtual ~GCSPanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void onStartExploration();

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void paramsCallback(const std_msgs::String::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Publisher trigger_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber params_sub_;

  QPushButton* start_button_;

  QLabel* status_label_;
  QLabel* params_label_;
  QLabel* odom_label_;
};

} // namespace rviz_gcs_plugin

#endif // GCS_PANEL_H