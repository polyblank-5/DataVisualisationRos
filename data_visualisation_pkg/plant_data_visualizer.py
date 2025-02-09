import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray # Replace with the appropriate message types for your topics
import json
import pygame
from typing import List, Tuple
import os 
import yaml
import threading
import random

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
GREEN = (0,255,0)
RED = (255,0,0)
ORANGE = (255,128,0)


class PlantDataVisualizer(Node):
    def __init__(self):
        super().__init__('plant_data_visualizer')

        os.chdir('src/data_visualisation_pkg/')
        #package_share_directory = os.path.join(    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join('config/', 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
        
        constants = config['constants']
        self._FRAME_HEIGHT = constants["PLANT_DATA_VISUALIZER"]['FRAME_HEIGHT']
        self._FRAME_WIDTH = constants['PLANT_DATA_VISUALIZER']['FRAME_WIDTH']
        self._SCREEN_WIDTH = constants['SCREEN_WIDTH']
        self._SCREEN_HEIGHT = constants['SCREEN_HEIGHT']
        self._FRAME_DISCRETIZATION = constants['FRAME_DISCRETIZATION']
        self._CELL_WIDTH = self._SCREEN_WIDTH / (self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)
        self._CELL_HEIGHT = self._SCREEN_HEIGHT / (self._FRAME_HEIGHT / self._FRAME_DISCRETIZATION)
        self._FONT_SIZE = constants['FONT_SIZE']

        pygame.init()
        self.screen = pygame.display.set_mode((self._SCREEN_WIDTH, self._SCREEN_HEIGHT))
        self.font = pygame.font.SysFont(None, 24)

        # Start a separate thread to handle the Pygame event loop
        self.pygame_thread = threading.Thread(target=self.pygame_loop)
        self.pygame_thread.daemon = True
        self.pygame_thread.start()
        
        # Subscription to the first topic
        self.plant_data_subscription = self.create_subscription(
            String,             # Replace 'String' with your topic's message type
            'plant_data_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_data_subscriber,    # Callback for the first topic
            10                  # QoS depth
        )
        self.plant_data_subscription  # Prevent unused variable warning

        self.plant_position_subscription = self.create_subscription(
            String,             # Replace 'String' with your topic's message type
            'plant_position_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_position_subscriber,    # Callback for the first topic
            10                  # QoS depth
        )
        self.plant_position_subscription

        self.plant_laser_position_subscription = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'plant_laser_data_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_laser_position_subscriber,    # Callback for the first topic
            10                  # QoS depth
        )
        self.plant_laser_position_subscription

        self.subscription_laser_position = self.create_subscription(
            Float32MultiArray,
            "laser_position_publisher",
            self.laser_position_callback,
            10
        )
        self.subscription_laser_position

         # Timer to run the simulation loop
        self.timer = self.create_timer(0.2, self.simulation_loop)  # 5 FPS

        # Initialize clock for Pygame
        self.clock = pygame.time.Clock()

        self.plant_positions:List[Tuple[float,float]] = []
        self.plant_positions_calc:List[Tuple[float,float]] = []
        self.plant_laser_positions:Tuple[float,float] = (0.0,0.0)
        self.laser_positions:Tuple[float,float] = (0.0,0.0)


    # Callback for the first topic
    def callback_plant_data_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_data_subscription.topic_name}: {msg.data}')
        self.plant_positions_calc = json.loads(msg.data)  

    def callback_plant_position_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_position_subscription.topic_name}: {msg.data}')
        self.plant_positions = json.loads(msg.data)

    def callback_plant_laser_position_subscriber(self, msg):
        #self.get_logger().info(f'Received from {self.plant_laser_position_subscription.topic_name}: {msg.data}')
        self.plant_laser_positions = (msg.data[0],msg.data[1])

    def laser_position_callback(self,msg:Float32MultiArray):
        self.laser_positions = (msg.data[0],msg.data[1])
        
    def draw_frame(self):
        """Draw the simulation frame, including grid, two outlined boxes, and circles."""
        self.screen.fill(WHITE)  # Clear the screen with a white background

        # Draw frame border
        pygame.draw.rect(self.screen, BLACK, (0, 0, self._SCREEN_WIDTH, self._SCREEN_HEIGHT), 2)

        # Draw grid
        for i in range(int(self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)):
            x = i * self._CELL_WIDTH
            pygame.draw.line(self.screen, GRAY, (x, 0), (x, self._SCREEN_HEIGHT))
        for i in range(int(self._FRAME_HEIGHT / self._FRAME_DISCRETIZATION)):
            y = i * self._CELL_HEIGHT
            pygame.draw.line(self.screen, GRAY, (0, y), (self._SCREEN_WIDTH, y))

        # Draw circles
        for weed in self.plant_positions_calc:
            pixel_x = int(weed[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
            pixel_y = int(weed[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
            pygame.draw.circle(self.screen, GREEN, (pixel_x, pixel_y), 5)

        for weed in self.plant_positions:
            pixel_x = int(weed[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
            pixel_y = int(weed[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
            pygame.draw.circle(self.screen, BLACK, (pixel_x, pixel_y), 5)

        plant_laser_area_x = int(self.plant_laser_positions[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
        plant_laser_area_y = int(self.plant_laser_positions[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
        pygame.draw.circle(self.screen, RED,(plant_laser_area_x,plant_laser_area_y),5)

        laser_x = int(self.laser_positions[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
        laser_y = int(self.laser_positions[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
        pygame.draw.circle(self.screen,ORANGE,(laser_x,laser_y),5)

        # Draw first box with only borders
        box1_width, box1_height = 140/ self._FRAME_DISCRETIZATION * self._CELL_WIDTH, 170/ self._FRAME_DISCRETIZATION * self._CELL_HEIGHT
        box1_y = 200 / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT
        box1_rect = pygame.Rect(
            (self._SCREEN_WIDTH // 2) - (box1_width // 2),  # Center x
            box1_y,  # y position
            box1_width,
            box1_height
        )
        pygame.draw.rect(self.screen, BLACK, box1_rect, 2)  # Border width = 2

        # Draw second box with only borders
        box2_width, box2_height = 200/ self._FRAME_DISCRETIZATION * self._CELL_WIDTH, 150/ self._FRAME_DISCRETIZATION * self._CELL_HEIGHT
        box2_y = 500 / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT
        box2_rect = pygame.Rect(
            (self._SCREEN_WIDTH // 2) - (box2_width // 2),  # Center x
            box2_y,  # y position
            box2_width,
            box2_height
        )
        pygame.draw.rect(self.screen, BLACK, box2_rect, 2)  # Border width = 2

        pygame.display.flip()

    def simulation_loop(self):
        """ROS-compatible simulation loop."""

        # Draw the frame (handled in the pygame event loop)
        self.draw_frame()

        # Cap the frame rate for ROS 2 interactions (not for Pygame display)
        self.clock.tick(5)

    def pygame_loop(self):
        """Run the Pygame event loop in a separate thread."""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.get_logger().info("Simulation terminated by user.")
                    pygame.quit()
                    rclpy.shutdown()

            # We let the simulation loop update the display
            pygame.time.wait(100)  # Slight delay to not hog CPU

def main(args=None):
    rclpy.init(args=args)
    node = PlantDataVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
