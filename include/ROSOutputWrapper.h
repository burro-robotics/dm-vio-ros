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

#ifndef DMVIO_ROS_ROSOUTPUTWRAPPER_H
#define DMVIO_ROS_ROSOUTPUTWRAPPER_H

#include <IOWrapper/Output3DWrapper.h>
#include <ros/ros.h>
#include <mutex>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>


namespace dmvio
{

// We publish 3 topics by default:
// dmvio/frame_tracked: DMVIOPoseMsg
// dmvio/unscaled_pose: PoseStamped
// dmvio/metric_poses: PoseStamped
// For more details on these see the README.md file.
class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
    ROSOutputWrapper();

    /*
     * Usage:
     * Called once after each keyframe is optimized and passes the new transformation from DSO frame (worldToCam in
     * DSO scale) to metric frame (imuToWorld in metric scale).
     * Use transformPose of the passed object to transform poses between the frames.
     * Note that the object should not be used any more after the method returns.
     * The caller can create copy however , preferable with the following constructor (as otherwise the shared_ptrs will be kept).
     * TransformDSOToIMU(TransformDSOToIMU& other, std::shared_ptr<bool> optScale, std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu);
     */
    virtual void publishTransformDSOToIMU(const dmvio::TransformDSOToIMU& transformDSOToIMU) override;

    /*
     * Usage:
     * Called every time the status of the system changes.
     */
    virtual void publishSystemStatus(dmvio::SystemStatus systemStatus) override;


    /* Usage:
     * Called once for each tracked frame, with the real-time, low-delay frame pose.
     *
     * Calling:
     * Always called, no overhead if not used.
     */
    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override;

    // In case you want to additionally publish pointclouds or keyframe poses you need to override Output3DWrapper::publishKeyframes
    bool TfGetTransform(tf::TransformListener& tf_listener,
                    const std::string & from_frame_id,
                    const std::string & to_frame_id,
                    const ros::Time & stamp,
                    tf::StampedTransform & transform,
                    double wait)
    {
        try
        {
            if(!stamp.isZero() || wait > 0.0)
            {
                std::string errorMsg;
                if(!tf_listener.waitForTransform(from_frame_id, to_frame_id, stamp, ros::Duration(wait), ros::Duration(0.01), &errorMsg))
                {
                    ROS_WARN("%s: Could not get transform from %s to %s (stamp=%f) after %f seconds "
                            "(\"wait_for_transform_duration\"=%f)! Error=\"%s\"",
                            ros::this_node::getName().c_str(),
                            from_frame_id.c_str(),
                            to_frame_id.c_str(),
                            stamp.toSec(),
                            wait,
                            wait,
                            errorMsg.c_str());

                    return false;
                }
                else
                {
                    tf_listener.lookupTransform(from_frame_id, to_frame_id, stamp, transform);
                    return true;
                }
            }
            return false;

        }
        catch(tf::TransformException & ex)
        {
            ROS_WARN( "%s",ex.what());
            return false;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher dmvioPosePublisher, systemStatePublisher, unscaledPosePublisher, metricPosePublisher, unscaledPosePublisher_odom, unscaled_pose_array;

    // Protects transformDSOToIMU.
    std::mutex mutex;

    std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
    bool scaleAvailable = false; // True iff transformDSOToIMU contains a valid scale.
    std::atomic<dmvio::SystemStatus> lastSystemStatus;
    tf::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tfbr_;
    bool convert{false};
    Sophus::SE3 world_to_odom;
    geometry_msgs::PoseArray parray_msg;


};


}
#endif //DMVIO_ROS_ROSOUTPUTWRAPPER_H
