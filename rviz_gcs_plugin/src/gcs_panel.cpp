#include "gcs_panel.h"
#include <QHBoxLayout>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_gcs_plugin::GCSPanel, rviz::Panel)

namespace rviz_gcs_plugin
{

GCSPanel::GCSPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  // Publishers
  trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/exploration/trigger", 1);
  finish_pub_ = nh_.advertise<std_msgs::Empty>("/exploration/finish", 1);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  // Subscriber
  odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &GCSPanel::odomCallback, this);

  // Layout
  QVBoxLayout* layout = new QVBoxLayout;

  start_button_ = new QPushButton("Start Exploration");
  stop_button_ = new QPushButton("Stop Exploration");
  goal_button_ = new QPushButton("Send Goal (Click in Map)");
  status_label_ = new QLabel("Status: Ready");
  params_label_ = new QLabel("Params: Loading...");
  odom_label_ = new QLabel("Odom: No data");

  // Read exploration params
  double center_x = 0.0, center_y = 0.0, width = 20.0, height = 20.0, spacing = 2.0;
  nh_.getParam("/exploration_node/exploration/center_x", center_x);
  nh_.getParam("/exploration_node/exploration/center_y", center_y);
  nh_.getParam("/exploration_node/exploration/width", width);
  nh_.getParam("/exploration_node/exploration/height", height);
  nh_.getParam("/exploration_node/exploration/line_spacing", spacing);
  params_label_->setText(QString("Center: (%.1f, %.1f) Size: %.1fx%.1f Spacing: %.1f")
                         .arg(center_x).arg(center_y).arg(width).arg(height).arg(spacing));

  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(goal_button_);
  layout->addWidget(status_label_);
  layout->addWidget(params_label_);
  layout->addWidget(odom_label_);

  setLayout(layout);

  // Connect signals
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartExploration()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopExploration()));
  connect(goal_button_, SIGNAL(clicked()), this, SLOT(onSendGoal()));
}

GCSPanel::~GCSPanel()
{
}

void GCSPanel::onStartExploration()
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 1.0;
  msg.pose.orientation.w = 1.0;
  trigger_pub_.publish(msg);
  status_label_->setText("Status: Exploration Started");
}

void GCSPanel::onStopExploration()
{
  std_msgs::Empty msg;
  finish_pub_.publish(msg);
  status_label_->setText("Status: Exploration Stopped");
}

void GCSPanel::onSendGoal()
{
  // For now, send a dummy goal. In a full implementation, you'd get the pose from RViz's clicked point.
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 5.0; // Example coordinates
  goal.pose.position.y = 0.0;
  goal.pose.position.z = 1.5;
  goal.pose.orientation.w = 1.0;

  goal_pub_.publish(goal);
  status_label_->setText("Status: Goal Sent");
}

void GCSPanel::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  odom_label_->setText(QString("Odom: (%.2f, %.2f, %.2f)").arg(x).arg(y).arg(z));
}

void GCSPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void GCSPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

} // namespace rviz_gcs_plugin