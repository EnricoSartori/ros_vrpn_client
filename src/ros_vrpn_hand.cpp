/*
# Copyright (c) 2014, Automatic Coordination of Teams Laboratory (ACT-Lab), 
#                     University of Souther California
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## author Wolfgang Hoenig (ACT-Lab, USC)
## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <reflex_msgs/HandCommand.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hand_tf_listener");
    ros::NodeHandle nh;

    ros::Publisher reflex_commander_pub = nh.advertise<reflex_msgs::HandCommand>("reflex_commander", 1000);

    tf::TransformListener listener;
    tf::StampedTransform transform_f1;
    tf::StampedTransform transform_f2;

    // To debug lookupTransform
    static tf::TransformBroadcaster br;
    tf::StampedTransform f1;
    tf::StampedTransform f2;
    tf::StampedTransform plane;

    ros::Rate rate(10.0);
    while (ros::ok()) {

        reflex_msgs::HandCommand msg;

        try{
            listener.lookupTransform("/plane", "/world", ros::Time(0), plane);
            listener.lookupTransform("/F1", "/plane", ros::Time(0), transform_f1);
            listener.lookupTransform("/F2", "/plane", ros::Time(0), transform_f2);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        /** 
        // Publish the transformation matrix to see it in rviz
        transform_f1.child_frame_id_ = "MYF1";
        transform_f2.child_frame_id_ = "MYF2";
        br.sendTransform(transform_f1);
        br.sendTransform(transform_f2);
        */
        
        // getAngle() : restituisce la componente theta della rappresentazione asse angolo della rotazione

        double angle_0 = transform_f1.getRotation().getAngle();
        double angle_1 = transform_f2.getRotation().getAngle();
        
        msg.angles[0] = angle_0;
        msg.angles[1] = angle_1;
        reflex_commander_pub.publish(msg); 

        printf("0: %f - 1: %f - 2: %f\n", msg.angles[0], msg.angles[1], msg.angles[2]);
    }
}
