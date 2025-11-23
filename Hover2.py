#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.srv import UpdateParams
from threading import Thread
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        # PID hesaplamaları
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix
        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.current_pose = None  # ORB_SLAM3'ten gelen pozisyon verisi
        self.last_pose_time = rospy.Time.now()
        self.pose_timeout = rospy.Duration(0.5)
        self.rate = rospy.Rate(10)

        # PID ayarları
        self.pid_x = PIDController(0.4, 0.05, 0.2)  # X ekseni PID parametreleri
        self.pid_y = PIDController(0.4, 0.05, 0.2)  # Y ekseni PID parametreleri
        self.pid_z = PIDController(1.25, 0.05, 0.5)  # Z ekseni PID parametreleri


        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)


        self.setParam("stabilizer/estimator", 2)
        self.setParam("stabilizer/controller", 1)
        self.setParam("commander/enHighLevel", 1)
        self.setParam("kalman/resetEstimation", 1)

        # ORB_SLAM3 verilerini dinleyen subscriber
        rospy.Subscriber("/ORB_SLAM3/orb_pose", PoseStamped, self.orb_pose_callback)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.seq=0
        self.msg.header.stamp=rospy.Time.now()
        self.msg.header.frame_id=worldFrame
        self.msg.yawrate=0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return 0.1
        elif distance < 0:
            return -0.1
        else:
            return 0

    def orb_pose_callback(self, data):
        self.current_pose = data.pose
        self.last_pose_time = rospy.Time.now()

    # x, y is the x, y distance relative to itself
    # z is absolute z distance
    # TODO: solve 0
    def goTo (self, x, y, zDistance, yaw):
        duration = 0
        duration_x = 0
        duration_y = 0
        duration_z = 0
        vx = 0
        vy = 0
        z = self.msg.zDistance # the zDistance we have before
        z_scale = self.getSpeed(z) # the z distance each time z has to increment, will be changed

        # for x, in secs
        if x != 0:
            duration_x = abs(x/0.1)
            vx = self.getSpeed(x)

        # for y, in secs
        if y != 0:
            duration_y = abs(y/0.1)
            vy = self.getSpeed(y)

        duration_z = abs(z-zDistance)/0.1
        durations = [duration_x, duration_y, duration_z]
        duration = max(durations)

        if duration == 0:
            return
        elif duration == duration_x:
            # x is the longest path
            vy *= abs(y/x)
            z_scale *= abs((z-zDistance)/x)
        elif duration == duration_y:
            # y is the longest path
            vx *= abs(x/y)
            z_scale *= abs((z-zDistance)/y)
        elif duration == duration_z:
            # z is the longest path
            vx *= abs(x/(z-zDistance))
            vy *= abs(y/(z-zDistance))

        print(vx)
        print(vy)
        print(z_scale)
        print(duration)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.msg.vx = vx
            self.msg.vy = vy
            self.msg.yawrate = 0.0
            self.msg.zDistance = z
            if z < zDistance:
                print(zDistance)
                print(z)
                z += z_scale
            else:
                z = zDistance
            now = rospy.get_time()
            if (now - start > duration):
                break
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(self.msg.vx)
            rospy.loginfo(self.msg.vy)
            rospy.loginfo(self.msg.yawrate)
            rospy.loginfo(self.msg.zDistance)
            self.pub.publish(self.msg)
            self.rate.sleep()

    def stabilize_hover(self, target_z):
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()  # PID hesaplaması için zaman farkı
            last_time = current_time

            # ORB_SLAM3 verileri varsa ve timeout süresini aşmamışsa PID kontrolü kullan
            if self.current_pose and (current_time - self.last_pose_time) < self.pose_timeout:
                # Hataları hesapla
                error_x = -self.current_pose.position.x
                error_y = -self.current_pose.position.y
                #error_z = target_z - self.current_pose.position.z
                error_z = target_z + (-self.current_pose.position.z)  # Z pozisyonunu pozitife çevir orbden - li artıyor ters...


                # PID denetleyiciyi kullanarak düzeltmeler yap
                self.msg.vx = self.pid_x.compute(error_x, dt)
                self.msg.vy = -0.3
                self.msg.yawrate = 150
                self.msg.zDistance = target_z + self.pid_z.compute(error_z, dt)
                rospy.loginfo("ORB_SLAM3 verileri ile PID kontrollü hover")
            else:
                # ORB_SLAM3 verisi yoksa varsayılan hover pozisyonunu koru
                rospy.logwarn("ORB_SLAM3 pose verisi yok, varsayılan stabilizasyon kullanılıyor")
                self.msg.vx = 0
                self.msg.vy = -0.3
                self.msg.yawrate = 150
                self.msg.zDistance = target_z

            # Hover komutunu yayınla
            self.pub.publish(self.msg)
            self.rate.sleep()

    def takeOff(self, zDistance):
        # Takeoff işlemi sırasında yükselmeyi sağla
        for _ in range(int(zDistance * 25)):  # Hedef yüksekliğe çıkana kadar artış sağla
            self.msg.vx = 0.0
            self.msg.vy = 0.0
            self.msg.yawrate = 0.0
            self.msg.zDistance = zDistance
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        # Hedef yüksekliğe ulaştıktan sonra stabilize hover fonksiyonunu çağır
        self.stabilize_hover(zDistance)

    def land(self):
        # İniş işlemi sırasında yüksekliği azaltarak indirme sağla
        zDistance = self.msg.zDistance

        while not rospy.is_shutdown() and zDistance > 0:
            self.msg.vx = 0.0
            self.msg.vy = 0.0
            self.msg.yawrate = 0.0
            self.msg.zDistance = zDistance
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.rate.sleep()
            zDistance -= 0.1  # İniş hızı

        self.stop_pub.publish(self.stop_msg)

def handler(cf):
    cf.takeOff(0.6)  # 0.6 metreye kalkış takeoff
    #cf.goTo(0.4, 0.1, 0.2, 0)
    cf.land()        # İniş

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    cf1 = Crazyflie("cf1")
    t1 = Thread(target=handler, args=(cf1,))
    t1.start()
