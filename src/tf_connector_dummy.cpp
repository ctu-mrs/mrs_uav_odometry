/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nodelet/nodelet.h>

// Msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>

// std
#include <string>
#include <mutex>

//}

namespace mrs_odometry
{

  class TFConnectorDummy : public nodelet::Nodelet
  {
  public:
    TFConnectorDummy() : m_node_name("TFConnectorDummy") {};
    std::string m_node_name;

  private:
    std::string m_connecting_frame_id;
    std::vector<std::string> m_root_frame_ids;
    std::vector<std::string> m_equal_frame_ids;

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

    ros::Subscriber m_sub_tf;
    ros::Publisher m_pub_tf;

  public:

    /* tf_callback() method //{ */

    void tf_callback(tf2_msgs::TFMessageConstPtr msg_ptr)
    {
      const tf2_msgs::TFMessage& tf_msg = *msg_ptr;
      std::vector<size_t> changed_frame_its; // changed frame ids of interest in this TF message
      for (const geometry_msgs::TransformStamped& tf : tf_msg.transforms)
      {
        // check whether this frame id is of interest
        for (size_t it = 0; it < m_equal_frame_ids.size(); it++)
        {
          const auto& trigger_frame_id = m_equal_frame_ids.at(it);
          if (tf.child_frame_id == trigger_frame_id)
            changed_frame_its.push_back(it);
        }
      }
    
      // if no interesting frame was changed, ignore the message
      if (changed_frame_its.empty())
        return;
    
      // otherwise create and publish an updated TF for each changed frame
      tf2_msgs::TFMessage new_tf_msg;
      new_tf_msg.transforms.reserve(changed_frame_its.size());
      for (const size_t it : changed_frame_its)
      {
        const auto& root_frame_id = m_root_frame_ids.at(it);
        const auto& equal_frame_id = m_equal_frame_ids.at(it);
        geometry_msgs::TransformStamped new_tf ;
        try
        {
          new_tf = m_tf_buffer.lookupTransform(equal_frame_id, root_frame_id, ros::Time(0));
        }
        catch (const tf2::TransformException& ex)
        {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", root_frame_id.c_str(), equal_frame_id.c_str(), ex.what());
          continue;
        }
        new_tf.child_frame_id = root_frame_id;
        new_tf.header.frame_id = m_connecting_frame_id;
        new_tf_msg.transforms.push_back(new_tf);
      }
    
      ROS_INFO_THROTTLE(3.0, "[TFConnectorDummy]: Publishing updated transform connection.");
      m_pub_tf.publish(new_tf_msg);
    }

    //}

    /* onInit() method //{ */

    virtual void onInit() override
    {
      ROS_INFO("[%s]: Initializing", m_node_name.c_str());
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      ros::Time::waitForValid();

      /* load parameters //{ */

      ROS_INFO("[%s]: LOADING STATIC PARAMETERS", m_node_name.c_str());
      mrs_lib::ParamLoader pl(nh, m_node_name);

      pl.load_param("connecting_frame_id", m_connecting_frame_id);
      pl.load_param("root_frame_ids", m_root_frame_ids);
      pl.load_param("equal_frame_ids", m_equal_frame_ids);

      if (!pl.loaded_successfully())
      {
        ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
        ros::shutdown();
      }

      if (m_root_frame_ids.size() != m_equal_frame_ids.size())
      {
        ROS_ERROR("[%s]: Number of root frame ids (%lu) must equal the number of equal frame ids (%lu), ending the node", m_node_name.c_str(), m_root_frame_ids.size(), m_equal_frame_ids.size());
        ros::shutdown();
      }

      //}

      /* publishers //{ */

      m_pub_tf = nh.advertise<tf2_msgs::TFMessage>("tf_out", 10);

      //}

      /* subscribers //{ */

      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
      m_sub_tf = nh.subscribe("tf_in", 10, &TFConnectorDummy::tf_callback, this);

      //}

      ROS_INFO("[%s]: Initialized", m_node_name.c_str());
    }

    //}

  };

}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::TFConnectorDummy, nodelet::Nodelet)

