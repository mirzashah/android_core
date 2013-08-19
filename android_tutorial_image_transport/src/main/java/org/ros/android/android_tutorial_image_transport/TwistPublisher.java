/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

package org.ros.android.android_tutorial_image_transport;

import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.ConnectedNode;
import org.ros.namespace.GraphName;
import java.util.Timer;
import java.util.TimerTask;
import geometry_msgs.Twist;
import android.util.Log;

public class TwistPublisher extends AbstractNodeMain
{
    private Twist            currentVelocityCommand_;
    private Publisher<Twist> driveCommandPublisher_;
    private boolean          isPublishEnabled_;

    public void isPublishEnabled(boolean isIt)
    /*************************************************************************/
    {
        isPublishEnabled_ = isIt;
    }

    public void setVelocity(double linearVelocityX, double linearVelocityY,
                                 double angularVelocityZ)
    /*************************************************************************/
    {
        currentVelocityCommand_.getLinear().setX(linearVelocityX);
        currentVelocityCommand_.getLinear().setY(-linearVelocityY);
        currentVelocityCommand_.getLinear().setZ(0);
        currentVelocityCommand_.getAngular().setX(0);
        currentVelocityCommand_.getAngular().setY(0);
        currentVelocityCommand_.getAngular().setZ(-angularVelocityZ);
    }

    @Override
    public GraphName getDefaultNodeName()
    /*************************************************************************/
    {
        return GraphName.of("ShieldTeleop/TwistPublisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode)
    /*************************************************************************/
    {
        super.onStart(connectedNode);
        Log.d("TwistPublisher", "TwistPublisher: Connected to ROS master. Creating publisher object...");
        driveCommandPublisher_ = connectedNode.newPublisher("/cmd_vel", geometry_msgs.Twist._TYPE);
        currentVelocityCommand_ = driveCommandPublisher_.newMessage();

        isPublishEnabled(true);

        setVelocity(0,0,0);

        Timer publisherTimer = new Timer();
        publisherTimer.schedule(new TimerTask() { //Note: This is a very interesting feature of Java, anonymous classes! Overriding a method for an object of type TimerTask without having to subclass explicitly!
            @Override
            public void run()
            {
                if(isPublishEnabled_)
                {
                    Log.d("TwistPublisher", "TwistPublisher: Publishing latest velocity command...");
                    driveCommandPublisher_.publish(currentVelocityCommand_);
                }
            }
        }, 0, 100);

    }

}
