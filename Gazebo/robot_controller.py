import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Obstacle Avoidance Node Started")
        self.state = "MOVING_FORWARD"  # Domyślny stan robota

    def scan_callback(self, scan):
        # Rozdziel dane na sektory: lewy, środkowy i prawy
        ranges = scan.ranges
        #left_sector = ranges[:len(ranges)//3]  # Lewa strona
        #front_sector = ranges[len(ranges)//3:2*len(ranges)//3]  # Przód
        #right_sector = ranges[2*len(ranges)//3:]  # Prawa strona
        left_sector = ranges[len(ranges)//6:3*len(ranges)//6]
        right_sector = ranges[3*len(ranges)//6:5*len(ranges)//6]
        front_sector = ranges[5*len(ranges)//6:]+ranges[:len(ranges)//6]

        # Znajdź minimalne odległości w sektorach
        min_left = min(left_sector) if left_sector else float('inf')
        min_front = min(front_sector) if front_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')

        self.get_logger().info(f"Distances - Left: {min_left}, Front: {min_front}, Right: {min_right}")

        self.avoid_obstacle(min_left, min_front, min_right)

    def avoid_obstacle(self, min_left, min_front, min_right):
        twist = Twist()

        # Logika unikania przeszkód
        if min_front < 0.3:  # Przeszkoda z przodu
            self.state = "TURNING"
            if min_left > min_right:
                twist.angular.z = 0.5  # Obrót w lewo
            else:
                twist.angular.z = -0.5  # Obrót w prawo
            twist.linear.x = 0.0
        else:
            # Jeśli nie ma przeszkód z przodu, poruszaj się do przodu
            self.state = "MOVING_FORWARD"
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

        if self.state == "TURNING":
            self.get_logger().info("Turning to avoid obstacle...")
        elif self.state == "MOVING_FORWARD":
            self.get_logger().info("Moving forward...")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

