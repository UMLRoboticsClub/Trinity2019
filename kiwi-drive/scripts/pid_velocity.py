#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
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
"""

import rospy
import roslib
import math

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from numpy import array

    
######################################################
######################################################
class PidVelocity():
######################################################
######################################################


    #####################################################
    def __init__(self):
    #####################################################
        rospy.init_node("pid_velocity")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.previous_error = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0
        
        ### get parameters #### 
        self.Kp = rospy.get_param('~Kp',0.01)
        self.Ki = rospy.get_param('~Ki',0)
        self.Kd = rospy.get_param('~Kd',0)
        self.out_min = rospy.get_param('~out_min',-1)
        self.out_max = rospy.get_param('~out_max',1)
        self.rate = rospy.get_param('~rate',30)
        self.rolling_pts = rospy.get_param('~rolling_pts',5)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4)
        self.ticks_per_meter = float(rospy.get_param('ticks_meter', 560 / (0.0825*math.pi)))  # The number of wheel encoder ticks per meter of travel
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001)
        self.encoder_min = rospy.get_param('encoder_min', -2147483647)
        self.encoder_max = rospy.get_param('encoder_max', 2147483647)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.wheel = rospy.get_param('~wheel', 'wheel')
        self.wheel_vtarget = rospy.get_param('~wheel_vtarget', 'wheel_vtarget')
        self.motor_cmd = rospy.get_param('~motor_cmd', 'motor_cmd')
        self.wheel_vel = rospy.get_param('~wheel_vel', 'wheel_vel')

        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = rospy.Time.now()
        rospy.loginfo("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f wheel:%s wheel_vtarget: %s motor_cmd: %s wheel_vel:%s" % (self.nodename, self.Kp, self.Ki, self.Kd, self.ticks_per_meter, self.wheel, self.wheel_vtarget, self.motor_cmd, self.wheel_vel))
        
        #### subscribers/publishers 
        rospy.Subscriber(self.wheel, Int32, self.wheelCallback) 
        rospy.Subscriber(self.wheel_vtarget, Float32, self.targetCallback) 
        self.pub_motor = rospy.Publisher(self.motor_cmd, Float32, queue_size=10) 
        self.pub_vel = rospy.Publisher(self.wheel_vel, Float32, queue_size=10)
        
    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()
        while not rospy.is_shutdown():
            self.spinOnce()
            try:
                self.r.sleep()
            except:
                pass
            
    #####################################################
    def spinOnce(self):
    #####################################################
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0 
        self.vel = 0.0
        #rospy.loginfo("spinning")
        
        # only do the loop if we've recently recieved a target velocity message
        while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
            self.calcVelocity()
            #rospy.loginfo("done calculating velocity...");
            self.doPid()
            #rospy.loginfo("motor cmd: {0}".format(self.motor))
            self.pub_motor.publish(self.motor)
            self.r.sleep()
            self.ticks_since_target += 1
            if self.ticks_since_target == self.timeout_ticks:
                self.pub_motor.publish(0)
            
    #####################################################
    def calcVelocity(self):
    #####################################################
        #rospy.loginfo("calculating velocity")
        self.dt_duration = rospy.Time.now() - self.then
        self.dt = self.dt_duration.to_sec()
        #rospy.loginfo("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" % (self.nodename, self.dt, self.wheel_latest, self.wheel_prev))
        rospy.loginfo("vel %.3f" % ((self.wheel_latest - self.wheel_prev)/self.dt))        
        if (self.wheel_latest == self.wheel_prev or abs(self.wheel_latest - self.wheel_prev)/(self.dt) > 1):
            # we haven't received an updated wheel lately
            self.calcRollingVel()
            cur_vel = self.vel  # if we got a tick right now, this would be the velocity
            if abs(cur_vel) < self.vel_threshold: 
                # if the velocity is < threshold, consider our velocity 0
                rospy.loginfo("-D- %s below threshold cur_vel=%0.3f vel=0" % (self.nodename, cur_vel))
                self.appendVel(0)
                self.calcRollingVel()
            else:
                rospy.loginfo("-D- %s above threshold cur_vel=%0.3f" % (self.nodename, cur_vel))
                if False: #abs(cur_vel) < self.vel:
                    rospy.logdebug("-D- %s cur_vel < self.vel" % self.nodename)
                    # we know we're slower than what we're currently publishing as a velocity
                    self.appendVel(cur_vel)
                    self.calcRollingVel()
            
        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.appendVel(cur_vel)
            self.calcRollingVel()
            #rospy.loginfo("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            self.wheel_prev = self.wheel_latest
            self.then = rospy.Time.now()
            
        #rospy.loginfo("publishing: %.3f" % self.vel)
        self.pub_vel.publish(self.vel)
        
    #####################################################
    def appendVel(self, val):
    #####################################################
        self.prev_vel.append(val)
        del self.prev_vel[0]
        
    #####################################################
    def calcRollingVel(self):
    #####################################################
        p = array(self.prev_vel)
        self.vel = p.mean()
        
    #####################################################
    def doPid(self):
    #####################################################
        pid_dt_duration = rospy.Time.now() - self.prev_pid_time
        pid_dt = pid_dt_duration.to_sec()
        self.prev_pid_time = rospy.Time.now()
        
        self.error = self.target - self.vel
        #rospy.loginfo(self.error)
        self.integral = self.integral + (self.error * pid_dt)
        #rospy.loginfo("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)" % (self.integral, self.integral, self.error, pid_dt))
        self.derivative = (self.error - self.previous_error) / pid_dt
        self.previous_error = self.error
        #print(self.motor, (self.Kp * self.error), (self.Ki * self.integral) , (self.Kd * self.derivative))
        self.motor += (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        #print(self.motor)
        if self.motor > self.out_max:
            self.motor = self.out_max
            self.integral = self.integral - (self.error * pid_dt)
        if self.motor < self.out_min:
            self.motor = self.out_min
            self.integral = self.integral - (self.error * pid_dt)
     	if abs(self.motor) < 0.25:
            self.motor = 0.25*self.motor/abs(self.motor) 
        if (self.target == 0):
            self.motor = 0
    
        #rospy.loginfo("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " % 
         #             (self.vel, self.target, self.error, self.integral, self.derivative, self.motor))
    
    


    #####################################################
    def wheelCallback(self, msg):
    ######################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
           
         
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter
        rospy.loginfo("%d / %d" % (enc, self.prev_encoder))
        self.prev_encoder = enc
        
        
#        rospy.logdebug("-D- %s wheelCallback msg.data= %0.3f wheel_latest = %0.3f mult=%0.3f" % (self.nodename, enc, self.wheel_latest, self.wheel_mult))
    
    ######################################################
    def targetCallback(self, msg):
    ######################################################
        self.target = msg.data
        self.ticks_since_target = 0
        #rospy.loginfo("-D- %s targetCallback " % (self.nodename))
    
    
if __name__ == '__main__':
    """ main """
    try:
        pidVelocity = PidVelocity()
        pidVelocity.spin()
    except rospy.ROSInterruptException:
        pass

