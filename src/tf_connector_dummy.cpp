/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

// std
#include <string>
#include <mutex>
#include <numeric>

//}

namespace mrs_uav_odometry
{

  class TFConnectorDummy : public nodelet::Nodelet
  {
  public:
    TFConnectorDummy() : m_node_name("TFConnectorDummy") {};
    std::string m_node_name;

  private:
    struct frame_connection_t
    {
      std::string root_frame_id;
      std::string equal_frame_id;
      tf2::Transform offset_tf;
      bool same_frames;
      ros::Time last_update;
    };

    std::string m_connecting_frame_id;
    using connection_vec_t = std::vector<std::shared_ptr<frame_connection_t>>;
    connection_vec_t m_frame_connections;

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

    ros::Subscriber m_sub_tf;
    ros::Publisher m_pub_tf;
    ros::Timer m_tim_tf;

    ros::Duration m_max_update_period = ros::Duration(0.1);

  public:

    /* update_tfs() method //{ */

    void update_tfs(const connection_vec_t& changed_connections)
    {
      // if changed_frame_its is empty, update all frames
      if (changed_connections.empty())
        return;

      const ros::Time now = ros::Time::now();

      // create and publish an updated TF for each changed frame
      tf2_msgs::TFMessage new_tf_msg;
      new_tf_msg.transforms.reserve(changed_connections.size());
      for (const auto& con_ptr : changed_connections)
      {
        const auto& root_frame_id = con_ptr->root_frame_id;
        const auto& equal_frame_id = con_ptr->equal_frame_id;
        geometry_msgs::TransformStamped new_tf;
        try
        {
          new_tf = m_tf_buffer.lookupTransform(equal_frame_id, root_frame_id, ros::Time(0));
        }
        catch (const tf2::TransformException& ex)
        {
          ROS_WARN_THROTTLE(1.0, "Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", root_frame_id.c_str(), equal_frame_id.c_str(), ex.what());
          continue;
        }

        // handle weird edge-cases like static transforms and transforms from-to the same frame
        if (new_tf.header.stamp == ros::Time(0) || new_tf.child_frame_id == new_tf.header.frame_id)
          new_tf.header.stamp = now;

        new_tf.child_frame_id = root_frame_id;
        new_tf.header.frame_id = m_connecting_frame_id;

        // apply the offset
        tf2::Transform tf;
        tf2::fromMsg(new_tf.transform, tf);
        tf = con_ptr->offset_tf * tf;
        new_tf.transform = tf2::toMsg(tf);

        new_tf_msg.transforms.push_back(new_tf);
        con_ptr->last_update = now;
      }
    
      if (!new_tf_msg.transforms.empty())
      {
        ROS_INFO_THROTTLE(1.0, "[TFConnectorDummy]: Publishing updated transform connection.");
        m_pub_tf.publish(new_tf_msg);
      }
    }

    //}

    /* tf_callback() method //{ */

    void tf_callback(tf2_msgs::TFMessageConstPtr msg_ptr)
    {
      const tf2_msgs::TFMessage& tf_msg = *msg_ptr;
      connection_vec_t changed_connections;
      for (const geometry_msgs::TransformStamped& tf : tf_msg.transforms)
      {
        // check whether this frame id is of interest
        for (const auto& con_ptr : m_frame_connections)
        {
          // skip connections that have the same equal and root frame
          // to avoid weird publish/callback loops
          if (con_ptr->same_frames)
            continue;

          const auto& trigger_frame_id = con_ptr->equal_frame_id;
          if (tf.child_frame_id == trigger_frame_id)
            changed_connections.push_back(con_ptr);
        }
      }
    
      // if no interesting frame was changed, ignore the message
      if (changed_connections.empty())
        return;
    
      update_tfs(changed_connections);
    }

    //}

    /* timer_callback() method //{ */
    void timer_callback([[maybe_unused]] const ros::TimerEvent&)
    {
      const ros::Time now = ros::Time::now();
    
      connection_vec_t changed_connections;
      for (const auto& con_ptr : m_frame_connections)
      {
        if (now - con_ptr->last_update > m_max_update_period)
          changed_connections.push_back(con_ptr);
      }
      update_tfs(changed_connections);
    }
    //}

    /* load_offsets() method //{ */
    std::vector<tf2::Transform> load_offsets(mrs_lib::ParamLoader& pl) const
    {
      std::vector<tf2::Transform> ret;
      const auto offsets_xml = pl.loadParam2<XmlRpc::XmlRpcValue>("offsets");
      if (offsets_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[%s]: The 'offsets' parameter doesn't have a type array, cannot load", m_node_name.c_str());
        return ret;
      }
    
      for (size_t it = 0; it < offsets_xml.size(); it++)
      {
        const auto offset = offsets_xml[it];
        if (offset.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("[%s]: The %lu-th member of the 'offsets' array is not an array, skipping", m_node_name.c_str(), it);
          continue;
        }
    
        if (offset.size() == 4)
        {
          const tf2::Vector3 translation(offset[0], offset[1], offset[2]);
          const Eigen::Quaterniond q(Eigen::AngleAxisd(offset[3], Eigen::Vector3d::UnitZ()));
          const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
          ret.emplace_back(rotation, translation);
        }
        else if (offset.size() == 7)
        {
          const tf2::Vector3 translation (offset[0], offset[1], offset[2]);
          // Eigen expects parameters of the constructor to be w, x, y, z
          const Eigen::Quaterniond q = Eigen::Quaterniond(offset[6], offset[3], offset[4], offset[5]).normalized();
          if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all())
          {
            ROS_ERROR_STREAM("[" << m_node_name << "]: The member of the 'offsets' array at index " << it << " has an invalid rotation (" << q.coeffs().transpose() << "), skipping");
            continue;
          }
          // tf2 expects parameters of the constructor to be x, y, z, w
          const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
          ret.emplace_back(rotation, translation);
        }
        else
        {
          ROS_ERROR("[%s]: The member of the 'offsets' array at index %lu has incorrect size (%d, has to be 4 or 7), skipping", m_node_name.c_str(), it, offset.size());
          continue;
        }
      }
    
      return ret;
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

      pl.loadParam("connecting_frame_id", m_connecting_frame_id);
      const auto root_frame_ids = pl.loadParam2<std::vector<std::string>>("root_frame_ids");
      const auto equal_frame_ids = pl.loadParam2<std::vector<std::string>>("equal_frame_ids");

      const auto offsets = load_offsets(pl);

      if (!pl.loadedSuccessfully())
      {
        ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
        ros::shutdown();
        return;
      }

      if (root_frame_ids.size() != equal_frame_ids.size()  || root_frame_ids.size() != offsets.size())
      {
        ROS_ERROR("[%s]: Number of root frame ids (%lu) must equal the number of equal frame ids (%lu) and the number of offsets (%lu), ending the node", m_node_name.c_str(), root_frame_ids.size(), equal_frame_ids.size(), offsets.size());
        ros::shutdown();
        return;
      }

      //}

      const auto now = ros::Time::now();
      for (size_t it = 0; it < root_frame_ids.size(); it++)
      {
        auto new_con_ptr = std::make_shared<frame_connection_t>();
        new_con_ptr->root_frame_id = root_frame_ids.at(it);
        new_con_ptr->equal_frame_id = equal_frame_ids.at(it);
        new_con_ptr->same_frames = new_con_ptr->root_frame_id == new_con_ptr->equal_frame_id;
        new_con_ptr->offset_tf = offsets.at(it);
        new_con_ptr->last_update = now;
        m_frame_connections.push_back(std::move(new_con_ptr));
      }

      /* publishers //{ */

      m_pub_tf = nh.advertise<tf2_msgs::TFMessage>("tf_out", 10);

      //}

      /* subscribers //{ */

      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
      m_sub_tf = nh.subscribe("tf_in", 10, &TFConnectorDummy::tf_callback, this);

      //}

      m_tim_tf = nh.createTimer(m_max_update_period, &TFConnectorDummy::timer_callback, this);

      ROS_INFO("[%s]: Initialized", m_node_name.c_str());
    }

    //}

  };

}  // namespace mrs_uav_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_odometry::TFConnectorDummy, nodelet::Nodelet)
