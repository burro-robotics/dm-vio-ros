/**
* ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutputWrapper.h"
#include <GTSAMIntegration/PoseTransformationIMU.h>
#include <std_msgs/Int32.h>
#include "dmvio_ros/DMVIOPoseMsg.h"
#include <util/FrameShell.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>




using namespace dmvio;

dmvio::ROSOutputWrapper::ROSOutputWrapper()
        : nh("dmvio")
{
    systemStatePublisher = nh.advertise<std_msgs::Int32>("system_status", 10);
    dmvioPosePublisher = nh.advertise<dmvio_ros::DMVIOPoseMsg>("frame_tracked", 10);
    unscaledPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("unscaled_pose", 10);
    unscaledPosePublisher_odom = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("unscaled_pose_odom", 10);

    // While we publish the metric pose for convenience we don't recommend using it.
    // The reason is that the scale used for generating it might change over time.
    // Usually it is better to save the trajectory and multiply all of it with the newest scale.
    metricPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("metric_pose", 10);
    unscaled_pose_array = nh.advertise<geometry_msgs::PoseArray>("unscaled_pose_array", 1);
}


void ROSOutputWrapper::publishTransformDSOToIMU(const TransformDSOToIMU& transformDSOToIMUPassed)
{
    std::unique_lock<std::mutex> lk(mutex);
    transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(transformDSOToIMUPassed,
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false));
    scaleAvailable = lastSystemStatus == SystemStatus::VISUAL_INERTIAL;
    // You could also publish the new scale (and potentially gravity direction) here already if you want to use it as
    // soon as possible. For this simple ROS wrapper I decided to publish it bundled with the newest tracked pose as
    // this is when it is usually needed.
}

void ROSOutputWrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
{
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(systemStatus);
    systemStatePublisher.publish(msg);
    lastSystemStatus = systemStatus;
}

void setMsgFromSE3(geometry_msgs::Pose& poseMsg, const Sophus::SE3d& pose)
{
    poseMsg.position.x = pose.translation()[0];
    poseMsg.position.y = pose.translation()[1];
    poseMsg.position.z = pose.translation()[2];
    poseMsg.orientation.x = pose.so3().unit_quaternion().x();
    poseMsg.orientation.y = pose.so3().unit_quaternion().y();
    poseMsg.orientation.z = pose.so3().unit_quaternion().z();
    poseMsg.orientation.w = pose.so3().unit_quaternion().w();
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib)
{
    dmvio_ros::DMVIOPoseMsg msg;
    msg.header.stamp = ros::Time(frame->timestamp);
    msg.header.frame_id = "world"; // burro doesn't know the transform to the world

    auto& camToWorld = frame->camToWorld;

    geometry_msgs::Pose& poseMsg = msg.pose;
    setMsgFromSE3(poseMsg, camToWorld);

    // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
    geometry_msgs::PoseWithCovarianceStamped unscaledMsg;
    unscaledMsg.header = msg.header;
    unscaledMsg.pose.pose = poseMsg;
    unscaledPosePublisher.publish(unscaledMsg);

    //initial_camera4_to_odom is equivalent to the transformation from world frame to odom
    //should only happen once
    tf::StampedTransform initial_world_to_odom;
    tf::StampedTransform from_camera_to_baselink_tf;
    if (!convert && TfGetTransform(tf_listener,
                        "odom",
                        "camera4_infra1_optical_frame", // where world to odom
                        ros::Time(0),
                        initial_world_to_odom,
                        3.0) && \
                        TfGetTransform(tf_listener,
                        "camera4_infra1_optical_frame",
                        "base_link",
                        ros::Time(0),
                        from_camera_to_baselink_tf,
                        3.0))
    {
        // camToWorld * from_odom_to_world
        convert = true;

        Eigen::Quaterniond q;
        q.w() = initial_world_to_odom.getRotation().getW();
        q.x() = initial_world_to_odom.getRotation().getX();
        q.y() = initial_world_to_odom.getRotation().getY();
        q.z() = initial_world_to_odom.getRotation().getZ();
        world_to_odom.so3().setQuaternion(q);
        world_to_odom.translation().x() = initial_world_to_odom.getOrigin().getX();
        world_to_odom.translation().y() = initial_world_to_odom.getOrigin().getY();
        world_to_odom.translation().z() = initial_world_to_odom.getOrigin().getZ();

        q.w() = from_camera_to_baselink_tf.getRotation().getW();
        q.x() = from_camera_to_baselink_tf.getRotation().getX();
        q.y() = from_camera_to_baselink_tf.getRotation().getY();
        q.z() = from_camera_to_baselink_tf.getRotation().getZ();
        camera_to_base_link.so3().setQuaternion(q);
        camera_to_base_link.translation().x() = from_camera_to_baselink_tf.getOrigin().getX();
        camera_to_base_link.translation().y() = from_camera_to_baselink_tf.getOrigin().getY();
        camera_to_base_link.translation().z() = from_camera_to_baselink_tf.getOrigin().getZ();
    }

    if(convert)
    {
        // publish
        geometry_msgs::PoseWithCovarianceStamped unscaledMsg_odom;
        parray_msg.header = unscaledMsg_odom.header = msg.header;
        // std::cout << "world to odom\n" << world_to_odom.matrix() << '\n';
        setMsgFromSE3(unscaledMsg_odom.pose.pose, world_to_odom * camToWorld * camera_to_base_link);
        parray_msg.poses.push_back(unscaledMsg_odom.pose.pose);
        unscaledMsg_odom.pose.covariance = {1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
        unscaledMsg_odom.header.frame_id = parray_msg.header.frame_id = "odom";
        unscaledPosePublisher_odom.publish(unscaledMsg_odom);
        unscaled_pose_array.publish(parray_msg);
    }

    {
        std::unique_lock<std::mutex> lk(mutex);

        if (!transformDSOToIMU)
        {
            std::cout << "No TransFromDSOToIMU\n";
        }

        if (!scaleAvailable)
        {
            std::cout << "No scaleAvailable\n";
        }
        if(transformDSOToIMU && scaleAvailable)
        {
            msg.scale = transformDSOToIMU->getScale();

            // Publish scaled pose.
            geometry_msgs::PoseStamped scaledMsg;
            scaledMsg.header = msg.header;

            // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
            // worldToCam as an input!
            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
            setMsgFromSE3(scaledMsg.pose, imuToWorld);

            metricPosePublisher.publish(scaledMsg);
        }else
        {
            msg.scale = std::numeric_limits<double>::quiet_NaN();
            if(transformDSOToIMU) assert(transformDSOToIMU->getScale() == 1.0);
        }

        if(transformDSOToIMU)
        {
            Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
            msg.rotationMetricToDSO.x = gravityDirection.unit_quaternion().x();
            msg.rotationMetricToDSO.y = gravityDirection.unit_quaternion().y();
            msg.rotationMetricToDSO.z = gravityDirection.unit_quaternion().z();
            msg.rotationMetricToDSO.w = gravityDirection.unit_quaternion().w();
            setMsgFromSE3(msg.imuToCam, transformDSOToIMU->getT_cam_imu());
        }
    }

    dmvioPosePublisher.publish(msg);
}

