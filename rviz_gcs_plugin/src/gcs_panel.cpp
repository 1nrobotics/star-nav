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

  // Subscriber
  odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &GCSPanel::odomCallback, this);
  params_sub_ = nh_.subscribe("/exploration/parameters", 1, &GCSPanel::paramsCallback, this);

  // Layout
  QVBoxLayout* layout = new QVBoxLayout;

  start_button_ = new QPushButton("Start Exploration");
  status_label_ = new QLabel("Status: Ready");
  params_label_ = new QLabel("Params: Waiting for exploration node...");
  odom_label_ = new QLabel("Odom: No data");

  layout->addWidget(start_button_);
  layout->addWidget(status_label_);
  layout->addWidget(params_label_);
  layout->addWidget(odom_label_);

  setLayout(layout);

  // Connect signals
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartExploration()));
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

void GCSPanel::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  odom_label_->setText(QString("Odom: (%1, %2, %3)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2));
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