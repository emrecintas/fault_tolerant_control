#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
import time

class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix
        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)
        self.setParam("kalman/resetEstimation", 1)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

        # PID parametreleri
        self.kp = 1.25
        self.ki = 0.05
        self.kd = 0.5
        self.error_sum = 0.0
        self.last_error = 0.0

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    def takeOff(self, zDistance):
        time_range = 1 + int(10 * zDistance / 0.4)
        for y in range(time_range):
            self.msg.vx = 0.0
            self.msg.vy = 0.0
            self.msg.yawrate = 0.0
            self.msg.zDistance = y / 25.0
            self.pub.publish(self.msg)
            self.rate.sleep()

    def hover_with_fault_tolerance(self, zDistance):
        start_time = time.time()
        hover_duration = 10.0  # Hover işlemi için en az 10 saniye

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time

            if elapsed >= 4.0 and elapsed < hover_duration:
                # 4. saniyede thrust değişikliği etki
                self.msg.vx = 0.0
                self.msg.vy = -0.3  # motorun etkisini azaltmak için
                self.msg.zDistance = zDistance - self.compute_error(zDistance)
                self.pub.publish(self.msg)
            elif elapsed >= hover_duration:
                # Hover süresi tamamlandığında inişe geç
                break
            else:
                # Normal hover işlemi
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.zDistance = zDistance
                self.pub.publish(self.msg)
            self.rate.sleep()

    def compute_error(self, target_z):
        current_z = self.msg.zDistance  # Mevcut yüksekliği takip ediyorum
        error = target_z - current_z
        self.error_sum += error
        error_delta = error - self.last_error
        self.last_error = error
        # PID dengeleme
        return self.kp * error + self.ki * self.error_sum + self.kd * error_delta

    def land(self):
        zDistance = self.msg.zDistance
        while zDistance > 0:
            self.msg.vx = 0.0
            self.msg.vy = 0.0
            self.msg.yawrate = 0.0
            self.msg.zDistance = zDistance
            self.pub.publish(self.msg)
            self.rate.sleep()
            zDistance -= 0.2
        self.stop_pub.publish(self.stop_msg)

def handler(cf):
    cf.takeOff(0.6)
    cf.hover_with_fault_tolerance(0.6)  # Hedef z mesafesinde hover işlemi
    cf.land()

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)

    cf1 = Crazyflie("cf1")
    t1 = Thread(target=handler, args=(cf1,))
    t1.start()
