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
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &GCSPanel::goalCallback, this);
  params_sub_ = nh_.subscribe("/exploration/parameters", 1, &GCSPanel::paramsCallback, this);

  // Layout
  QVBoxLayout* layout = new QVBoxLayout;

  start_button_ = new QPushButton("Start Exploration");
  stop_button_ = new QPushButton("Stop Exploration");
  goal_button_ = new QPushButton("Send Goal (Click in Map)");
  status_label_ = new QLabel("Status: Ready");
  params_label_ = new QLabel("Params: Waiting for exploration node...");
  odom_label_ = new QLabel("Odom: No data");
  goal_label_ = new QLabel("Goal: None");

  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(goal_button_);
  layout->addWidget(status_label_);
  layout->addWidget(params_label_);
  layout->addWidget(odom_label_);
  layout->addWidget(goal_label_);

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
  odom_label_->setText(QString("Odom: (%1, %2, %3)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2));
}

void GCSPanel::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  goal_label_->setText(QString("Goal: (%1, %2, %3)").arg(x, 0, 'f', 1).arg(y, 0, 'f', 1).arg(z, 0, 'f', 1));
}

void GCSPanel::paramsCallback(const std_msgs::String::ConstPtr& msg)
{
  params_label_->setText(QString::fromStdString(msg->data));
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