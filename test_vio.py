import rospy
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF
import tf

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        
        self.pose_sub = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)
        self.slam_pose_sub = rospy.Subscriber('/slam/pose', PoseStamped, self.slam_pose_callback)
        
        self.cmd_pub = rospy.Publisher('/cf1/cmd_vel', Twist, queue_size=10)
        
        self.current_pose = PoseStamped()
        self.slam_pose = PoseStamped()
        self.corrected_pose = PoseStamped()
        
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 1.0
        self.target_pose.pose.position.y = 1.0
        self.target_pose.pose.position.z = 1.0

        # PID kontrol parametreleri
        self.kp_xy = 2.0
        self.kp_z = 2.0
        self.ki_xy = 0.5
        self.ki_z = 0.5
        self.kd_xy = 0.1
        self.kd_z = 0.1
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

        # EKF ayarları
        self.ekf = EKF(dim_x=6, dim_z=3)
        self.ekf.x = np.array([0., 0., 0., 0., 0., 0.])
        self.ekf.F = np.array([[1,0,0,1,0,0],
                               [0,1,0,0,1,0],
                               [0,0,1,0,0,1],
                               [0,0,0,1,0,0],
                               [0,0,0,0,1,0],
                               [0,0,0,0,0,1]])
        self.ekf.H = np.array([[1,0,0,0,0,0],
                               [0,1,0,0,0,0],
                               [0,0,1,0,0,0]])
        self.ekf.P *= 1000.
        self.ekf.R = np.array([[5,0,0],
                               [0,5,0],
                               [0,0,5]])
        self.ekf.Q = np.array([[0.1,0,0,0,0,0],
                               [0,0.1,0,0,0,0],
                               [0,0,0.1,0,0,0],
                               [0,0,0,0.1,0,0],
                               [0,0,0,0,0.1,0],
                               [0,0,0,0,0,0.1]])

        # Tamamlayıcı Filtre Parametreleri
        self.alpha_yaw = 0.98

        # Ağırlıklandırma parametreleri
        self.weight_imu = 0.8
        self.weight_slam = 0.2

    def pose_callback(self, msg):
        self.current_pose = msg
        self.update_ekf()

    def slam_pose_callback(self, msg):
        self.slam_pose = msg
        self.update_ekf()

    def update_ekf(self):
        if self.current_pose and self.slam_pose:
            z = np.array([self.slam_pose.pose.position.x,
                          self.slam_pose.pose.position.y,
                          self.slam_pose.pose.position.z])
            
            # EKF güncelleme
            self.ekf.predict()
            self.ekf.update(z, HJacobian=self.jacobian, Hx=self.hx)
            
            # Ağırlıklandırılmış pozisyon tahmini
            self.corrected_pose.pose.position.x = self.weight_imu * self.current_pose.pose.position.x + self.weight_slam * self.ekf.x[0]
            self.corrected_pose.pose.position.y = self.weight_imu * self.current_pose.pose.position.y + self.weight_slam * self.ekf.x[1]
            self.corrected_pose.pose.position.z = self.weight_imu * self.current_pose.pose.position.z + self.weight_slam * self.ekf.x[2]
            
            # Yaw verisini tamamlayıcı filtre ile birleştir
            imu_orientation = self.current_pose.pose.orientation
            slam_orientation = self.slam_pose.pose.orientation
            _, _, imu_yaw = tf.transformations.euler_from_quaternion([imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w])
            _, _, slam_yaw = tf.transformations.euler_from_quaternion([slam_orientation.x, slam_orientation.y, slam_orientation.z, slam_orientation.w])

            combined_yaw = self.alpha_yaw * imu_yaw + (1 - self.alpha_yaw) * slam_yaw

            combined_quat = tf.transformations.quaternion_from_euler(0, 0, combined_yaw)
            self.corrected_pose.pose.orientation.x = combined_quat[0]
            self.corrected_pose.pose.orientation.y = combined_quat[1]
            self.corrected_pose.pose.orientation.z = combined_quat[2]
            self.corrected_pose.pose.orientation.w = combined_quat[3]

    def hx(self, x):
        return np.array([x[0], x[1], x[2]])

    def jacobian(self, x):
        return np.array([[1,0,0,0,0,0],
                         [0,1,0,0,0,0],
                         [0,0,1,0,0,0]])

    def update(self):
        cmd = Twist()
        
        error_x = self.target_pose.pose.position.x - self.corrected_pose.pose.position.x
        error_y = self.target_pose.pose.position.y - self.corrected_pose.pose.position.y
        error_z = self.target_pose.pose.position.z - self.corrected_pose.pose.position.z

        # PID kontrol hesaplamaları
        self.integral_x += error_x
        self.integral_y += error_y
        self.integral_z += error_z

        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y
        derivative_z = error_z - self.prev_error_z

        cmd.linear.x = self.kp_xy * error_x + self.ki_xy * self.integral_x + self.kd_xy * derivative_x
        cmd.linear.y = self.kp_xy * error_y + self.ki_xy * self.integral_y + self.kd_xy * derivative_y
        cmd.linear.z = self.kp_z * error_z + self.ki_z * self.integral_z + self.kd_z * derivative_z

        # Oryantasyon verilerini kullanarak drone'u döndürme
        cmd.angular.z = self.corrected_pose.pose.orientation.z  # Yaw kontrolü

        self.cmd_pub.publish(cmd)

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    drone_controller = DroneController()
    drone_controller.run()
