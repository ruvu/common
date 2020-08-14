//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./configurable_pose_tool.hpp"

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

#include <QMessageBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <string>
#include <vector>

namespace ruvu_rviz_plugins
{
const char* TYPE = "geometry_msgs/PoseStamped";

//!
//! \brief getTopics Get the topics of a specific type within a namespace
//! \param ns Namespace
//! \param type Topic type
//! \return List of topic names
//!
std::vector<std::string> getTopics(const std::string& ns, const std::string type)
{
  ROS_INFO("Querying master for topics of geometry_msgs/PoseStamped within namespace %s", ns.c_str());

  //  NOTE: This call did not work
  //  ros::master::V_TopicInfo master_topics;
  //  ros::master::getTopics(master_topics);

  std::vector<std::string> topics;

  XmlRpc::XmlRpcValue params("ros_topic_list");
  XmlRpc::XmlRpcValue results;
  XmlRpc::XmlRpcValue r;

  if (ros::master::execute("getTopicTypes", params, results, r, false) == true)
  {
    if (results.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      int32_t i = 2;
      if (results[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int32_t j = 0; j < results[i].size(); ++j)
        {
          if (results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            if (results[i][j].size() == 2)
            {
              if (results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString &&
                  results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString)
              {
                std::string topic = static_cast<std::string>(results[i][j][0]);
                std::string datatype = static_cast<std::string>(results[i][j][1]);
                if (topic.find(ns) == 0 && datatype == type)
                {
                  ROS_INFO("Found topic %s with type %s in namespace %s", topic.c_str(), type.c_str(), ns.c_str());
                  topics.push_back(topic);
                }
              }
            }
          }
        }
      }
    }
  }

  return topics;
}

//!
//! \brief The TopicSelectionDialog class for selection of a topic
//!
class TopicSelectionDialog : public QDialog
{
public:
  TopicSelectionDialog(const QStringList& items, int selected_index)
  {
    setLayout(new QHBoxLayout());

    box = new QComboBox;
    box->addItems(items);
    if (selected_index < items.size())
    {
      box->setCurrentIndex(selected_index);
    }
    layout()->addWidget(box);

    QPushButton* ok = new QPushButton("Publish");
    layout()->addWidget(ok);
    connect(ok, &QPushButton::clicked, this, [this]() { accept(); });  // NOLINT
  }

  QComboBox* comboBox()
  {
    return box;
  }

private:
  QComboBox* box;
};

ConfigurablePoseTool::ConfigurablePoseTool() : selected_index_(0)
{
  auto msg = QString("This namespace will be scanned for topics with type ") + TYPE +
             ". The user can select the desired "
             "topic when a pose has been set.";
  namespace_ = new rviz::StringProperty("Pose Stamped namespace", "/", msg, getPropertyContainer(),
                                        SLOT(updatePublishers()), this);
}

void ConfigurablePoseTool::updatePublishers()
{
  publisher_map_.clear();

  std::vector<std::string> topics = getTopics(namespace_->getStdString(), TYPE);
  if (topics.empty())
  {
    auto msg = QString("No topics of type ") + TYPE + " found within namespace " + namespace_->getString() + "!";
    QMessageBox::warning(0, QString("No topics found!"), msg);
  }
  else
  {
    for (auto topic : topics)
    {
      publisher_map_[topic] = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);
    }
  }
}

void ConfigurablePoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Configurable 2D nav goal");
  updatePublishers();
}

void ConfigurablePoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);

  QStringList item_list;
  for (auto it : publisher_map_)
  {
    item_list.push_back(it.first.c_str());
  }

  TopicSelectionDialog dialog(item_list, selected_index_);
  if (dialog.exec() == QDialog::Accepted)
  {
    selected_index_ = dialog.comboBox()->currentIndex();

    ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
             fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
             goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
    publisher_map_[dialog.comboBox()->currentText().toStdString()].publish(goal);
  }
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::ConfigurablePoseTool, rviz::Tool)
