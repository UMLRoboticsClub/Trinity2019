#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rospy
import roslib
#roslib.load_manifest('differential_drive')
import math

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 560 / (0.0825*math.pi)))  # The number of wheel encoder ticks per meter of travel
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483647)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )

        self.fwheel = rospy.get_param('~encoder1', 'enc1')
        self.swheel = rospy.get_param('~encoder2', 'enc2')
        self.twheel = rospy.get_param('~encoder3', 'enc3') 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
       
        print(self.fwheel)
        print(self.swheel)
        print(self.twheel)
        print(self.base_frame_id)
        print(self.odom_frame_id)

        # internal data
        self.enc_first = None        # wheel encoder readings
        self.enc_second = None
        self.enc_third = None
        self.first = 0               # actual values coming back from robot
        self.second = 0
        self.third = 0
        self.fmult = 0
        self.smult = 0
        self.tmult = 0
        self.prev_fencoder = 0
        self.prev_sencoder = 0
        self.prev_tencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber(self.fwheel, Int32, self.fwheelCallback)
        rospy.Subscriber(self.swheel, Int32, self.swheelCallback)
        rospy.Subscriber(self.twheel, Int32, self.twheelCallback)

        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_first is None:
                d_first = 0
                d_second = 0
                d_third = 0
            else:
                d_first = (self.first - self.enc_first) / self.ticks_meter
                d_second = (self.second - self.enc_second) / self.ticks_meter
                d_third = (self.third - self.enc_third) / self.ticks_meter
            self.enc_first = self.first
            self.enc_second = self.second
            self.enc_third = self.third
            #print("ticks per meter: {0}".format(self.ticks_meter))
            #print("distances: {0}, {1}, {2}".format(d_first, d_second, d_third))
    
            # distance traveled in x an y, rotation in z 
            x = (-2 * d_second + d_first + d_third) / 3
            y = (d_first - d_third) * math.sqrt(3) / -3
            th = ((d_first + d_second + d_third) / 3) / 0.113
            self.th += th
            #print("theta: {0}".format(self.th))
            #print("dist before ({0}, {1})".format(x, y))
            new_x = x * math.cos(self.th) - y * math.sin(self.th)
            new_y = x * math.sin(self.th) + y * math.cos(self.th)
            #print("dist after ({0}, {1})".format(x, y))
            self.x += new_x
            self.y += new_y

            #print("x dist: {0}, y dist: {1}".format(self.x, self.y))
            #print("time elapsed: {0}".format(elapsed))
            
            # calculate velocities
            self.dx = x / elapsed
            self.dy = y / elapsed
            self.dr = th / elapsed
          
            #print("velocities: {0}, {1}, {2}".format(self.dx, self.dy, self.dr))
             
           # if (d != 0):
            #    # calculate distance traveled in x and y
            #    x = cos( th ) * d
           #     y = -sin( th ) * d
           #     # calculate the final position of the robot
           #     self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
           #     self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
          #  if( th != 0):
           #     self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = math.sin( self.th / 2 )
            quaternion.w = math.cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            #print("sent transform from %s to %s" % (self.base_frame_id, self.odom_frame_id))
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = self.dy
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            #print("robot pose is: ", self.x, self.y)            
            


    #############################################################################
    def fwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_fencoder > self.encoder_high_wrap):
            self.fmult = self.fmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_fencoder < self.encoder_low_wrap):
            self.fmult = self.fmult - 1
            
        self.first = 1.0 * (enc + self.fmult * (self.encoder_max - self.encoder_min)) 
        self.prev_fencoder = enc
        
    #############################################################################
    def swheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_sencoder > self.encoder_high_wrap):
            self.smult = self.smult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_sencoder < self.encoder_low_wrap):
            self.smult = self.smult - 1
            
        self.second = 1.0 * (enc + self.smult * (self.encoder_max - self.encoder_min))
        self.prev_sencoder = enc

    #############################################################################
    def twheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_tencoder > self.encoder_high_wrap):
            self.tmult = self.tmult + 1

        if(enc > self.encoder_high_wrap and self.prev_tencoder < self.encoder_low_wrap):
            self.tmult = self.tmult - 1

        self.third = 1.0 * (enc + self.tmult * (self.encoder_max - self.encoder_min))
        self.prev_tencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass

