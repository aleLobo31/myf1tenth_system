# Import ROS libraries and useful classes
import rclpy
from rclpy.node import Node
#from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from nav_msgs.msg import Odometry

# Import other libraries
import numpy as np
import transforms3d.euler

# Import other required libraries
class ParticleFilter(Node):
    def __init__(self):
        super().__init__('my_particle_filter')

        # Aux Variables
        self.aux = True

        # Parameter Declaration
        self.declare_parameter('odom_topic', "odom")
        self.declare_parameter('max_particles', 500)

        # Retrieve Parameter Values
        self.odom_topic = self.get_parameter('odom_topic').value
        self.MAX_PARTICLES = self.get_parameter('max_particles').value

        # We define particles as rows of a matrix with 3 dimensions (x, y, orientation)
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES)
        self.pose = np.zeros(3) # Pose = [x, y, orientation]
        self.last_pose = np.zeros(3)

        # We create required subscribers and publishers
        self.odom_sub_ = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            1
        )

        self.particle_pub_ = self.create_publisher(PoseArray, 'pf/viz/particles', 1)

        self.get_logger().info("Node Created")

    @staticmethod
    def rotation_matrix(theta) -> np.matrix:
        c, s = np.cos(theta), np.sin(theta)
        return np.matrix([[c, -s], [s, c]])
    
    @staticmethod
    def quaternion_to_angle(q):
        '''Transform a quaternion into an angle in radians'''
        quat = [q.w, q.x, q.y, q.z]
        euler_angles = transforms3d.euler.quat2euler(quat, axes='sxyz')
        return euler_angles[2]

    @staticmethod
    def angle_to_quaternion(angle):
        '''Transform an angle in radians into a quaternion'''
        euler_angles = (0.0, 0.0, angle)
        q = transforms3d.euler.euler2quat(*euler_angles, axes='sxyz')
        q_out = Quaternion()
        q_out.x = q[1]
        q_out.y = q[2]
        q_out.z = q[3]
        q_out.w = q[0]
        return q_out

    def particle_to_pose(self, particle):
        '''Tranform a particle [x, y, theta] into a Pose Object'''
        aux_pose = Pose()
        aux_pose.position.x = particle[0]
        aux_pose.position.y = particle[1]
        aux_pose.orientation = self.angle_to_quaternion(particle[2])
        return aux_pose

    def initialize_particles(self):
        # Set random values for x coordinate of each particle
        self.particles[:, 0] = self.pose[0] + np.random.normal(loc=0.0, scale=0.25, size=self.MAX_PARTICLES) 
        
        # Set random values for y coordinate of each particle
        self.particles[:, 1] = self.pose[1] + np.random.normal(loc=0.0, scale=0.25, size=self.MAX_PARTICLES)
        
        # Set random values for orientation of each particle
        self.particles[:, 2] = self.pose[2] + np.random.normal(loc=0.0, scale=0.25, size=self.MAX_PARTICLES)

        return

    def particle_visualization(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = '/map'
        pose_array.poses = list(map(self.particle_to_pose, self.particles))
        self.particle_pub_.publish(pose_array)
        return

    def motion_model(self, action: np.array):
        '''
        Propagamos la acción a las partículas, para ello hay que aplicar una
        matriz de Rotación definida por su ángulo con respecto al marco de referencia
        del coche
        '''
        cosines = np.cos(self.particles[:,2])
        sines = np.sin(self.particles[:,2])

        self.particles[:,0] += cosines*action[0] - sines*action[1] #+ np.random.normal(loc=0.0, scale=0.05, size=self.MAX_PARTICLES)
        self.particles[:,1] += sines*action[0] + cosines*action[1] #+ np.random.normal(loc=0.0, scale=0.025, size=self.MAX_PARTICLES)
        self.particles[:,2] += action[2]
        return
    
    def sensor_model(self):
        pass

    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = self.quaternion_to_angle(msg.pose.pose.orientation)
        speed = (pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2))**(1/2)

        # self.get_logger().info(f'Pose -> x = {self.pose[0]}, y = {self.pose[1]}, theta = {self.pose[2]}')

        if(self.aux == True):
            # Initialize a sample of particles
            self.initialize_particles()

            self.aux = False

        if(speed > 0):
            '''
            No se puede definir directamente la variación de posición a las partículas
            porque estas están rotadas respecto al pose anterior.
            '''
            rotation_matrix = self.rotation_matrix(self.last_pose[2])
            position_variation = np.array([[self.pose[0] - self.last_pose[0]], [self.pose[1] - self.last_pose[1]]])
            transformation = rotation_matrix*position_variation
            action = np.array([transformation[0,0], transformation[1,0], self.pose[2] - self.last_pose[2]])
            
            # self.get_logger().info(f"R: {rotation_matrix}")
            # self.get_logger().info(f"Delta {position_variation}")
            # self.get_logger().info(f"T {transformation}")

            self.motion_model(action)

        self.particle_visualization()

        self.last_pose[0] = self.pose[0]
        self.last_pose[1] = self.pose[1]
        self.last_pose[2] = self.pose[2]
        return

def main(args=None):    
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    pf.destroy_node()
    rclpy.shutdown()