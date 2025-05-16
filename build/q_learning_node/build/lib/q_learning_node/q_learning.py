import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class QLearningNode(Node):
    def __init__(self):
        super().__init__('q_learning_node')
        
        self.lidar_sub = self.create_subscription(
            Float32MultiArray, 'lidar_val', self.lidar_callback, 10)
        
        self.publisher = self.create_publisher(
            Int32MultiArray, 'path_data', 10)
        
        self.grid_size = 7  # 3m x 3m divided into 25cm cells
        self.q_table = np.zeros((self.grid_size, self.grid_size), dtype=[('action', 'float64', 4), ('obstacle', 'bool')])
        self.alpha = 0.1  # Learning rate
        self.gamma = 0.9  # Discount factor
        self.epsilon = 1.0  # Initial exploration rate
        self.epsilon_min = 0.01  # Minimum exploration rate
        self.epsilon_decay = 0.99  # Decay factor
        self.timeout_count=25
        self.action_no=0
        self.robot_position = (0, 0)
        self.goal_position = (7, 7)
        self.action = None
        self.episode_path = []  # Store actions for replay
        self.returning = False  # Flag for returning to start
        self.new_position = [0,0]
        self.max_episodes = 200  # Set the number of episodes
        self.current_episode = 0
        self.get_logger().info("Q-Learning node Started")
        
    def lidar_callback(self, msg):
        self.robot_position = self.new_position
        lidar_data = msg.data
        self.get_logger().info("Hi2")
        self.process_lidar_data(lidar_data)
        self.take_action()
    
    def process_lidar_data(self, lidar_data):
        for i in range(0,len(lidar_data),2):
            [angle,distance]=[lidar_data[i],lidar_data[i+1]]
            if distance < 0.5: #upto 50cm
                obs_x = self.robot_position[0] + math.ceil((distance * np.cos(np.radians(angle)))*4)
                obs_y = self.robot_position[1] - math.ceil((distance * np.sin(np.radians(angle)))*4)
                if 0 <= obs_x < self.grid_size and 0 <= obs_y < self.grid_size:
                    self.q_table[obs_x, obs_y]['obstacle'] = True
                    #self.get_logger().info(f"Obstacle in {obs_x}, {obs_y}")
                    
    def choose_action(self):
        valid_action_found = False
        while not valid_action_found:
            if np.random.rand() < self.epsilon:
                action = np.random.choice(4)
            else:
                action = np.argmax(self.q_table[self.robot_position]['action'])
            self.new_position = self.get_new_position(self.robot_position, action)
            if self.is_valid_position(self.new_position):
                valid_action_found = True
                self.get_logger().info("Valid!!")
                #self.get_logger().info("Invalid!!")
        return action
    
    def take_action(self):
        if not self.returning:
            self.action = self.choose_action()
            self.episode_path.append(self.action)
            self.action_no += 1
        else:
            if self.episode_path:
                self.action = self.episode_path.pop()
            else:
                self.reset_episode()
                return

        msg = Int32MultiArray() if not self.returning else 0 
        msg.data = self.new_position
        self.publisher.publish(msg)
        self.get_logger().info(f"{msg.data}")
        
        if not self.returning:
            reward = self.get_reward()
            self.update_q_table(reward)  
            if self.robot_position == self.goal_position or self.action_no==self.timeout_count:
                self.returning = True #robot should rotate 180 degrees here
    
    def get_new_position(self, position, action):
        moves = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        dx, dy = moves[action]
        return [position[0] + dx, position[1] + dy]
    
    def is_valid_position(self, position):
        x, y = position
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size and not self.q_table[x,y]['obstacle']
    
    def get_reward(self):
        if self.new_position == self.goal_position:
            return 100
       #min_dist = np.min(self.q_table[self.robot_position])
        if self.q_table[tuple(self.new_position)]['obstacle']:
            return -10
        return -1
    
    def update_q_table(self, reward):
        #next_action = self.choose_action()
        self.q_table[self.robot_position[0],self.robot_position[1]]['action'][self.action] += self.alpha * (
            reward + self.gamma * np.max(self.q_table[self.new_position]['action']) - self.q_table[self.robot_position[0],self.robot_position[1]]['action'][self.action]
        )
        self.get_logger().info(f"{self.q_table}")
    
    def reset_episode(self):
        if self.current_episode >= self.max_episodes:
            self.get_logger().info("Training Completed after 200 episodes.")
            rclpy.shutdown()
            return
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        self.current_episode += 1
        #self.robot_position = (0, 0) (Not required since it is already at (0,0)
        self.returning = False
        self.episode_path = []
        self.get_logger().info(f"New Episode {self.current_episode} Started") 
        self.action_no = 0#Robot should rotate 180 degrees


def main(args=None):
    rclpy.init(args=args)
    node = QLearningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

