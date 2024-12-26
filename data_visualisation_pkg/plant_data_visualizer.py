import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with the appropriate message types for your topics
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


class PlantDataVisualizer(Node):
    def __init__(self):
        super().__init__('plant_data_visualizer')

        os.chdir('src/data_collection_pkg')
        #package_share_directory = os.path.join(    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join('config/', 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
        
        constants = config['constants']
        self._FRAME_HEIGHT = constants['FRAME_HEIGHT']
        self._FRAME_WIDTH = constants['FRAME_WIDTH']
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

         # Timer to run the simulation loop
        self.timer = self.create_timer(0.2, self.simulation_loop)  # 5 FPS

        # Initialize clock for Pygame
        self.clock = pygame.time.Clock()

        self.plant_positions:List[Tuple[float,float]] = []
        self.plant_positions_calc:List[Tuple[float,float]] = []


    # Callback for the first topic
    def callback_plant_data_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_data_subscription.topic_name}: {msg.data}')
        self.plant_positions_calc = json.loads(msg.data)  

    def callback_plant_position_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_position_subscription.topic_name}: {msg.data}')
        self.plant_positions = json.loads(msg.data)
        
    def draw_frame(self):
        """Draw the simulation frame, including weeds and numbering."""
        self.screen.fill(WHITE)
        pygame.draw.rect(self.screen, BLACK, (0, 0, self._SCREEN_WIDTH, self._SCREEN_HEIGHT), 2)

        # Draw grid
        for i in range(int(self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)):
            x = i * self._CELL_WIDTH
            pygame.draw.line(self.screen, GRAY, (x, 0), (x, self._SCREEN_HEIGHT))
        for i in range(int(self._FRAME_HEIGHT / self._FRAME_DISCRETIZATION)):
            y = i * self._CELL_HEIGHT
            pygame.draw.line(self.screen, GRAY, (0, y), (self._SCREEN_WIDTH, y))

        # Draw weeds
        for weed in self.plant_positions_calc:
            pixel_x = int(weed[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
            pixel_y = int(weed[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
            pygame.draw.circle(self.screen, GREEN, (pixel_x, pixel_y), 5)
            #id_text = self.font.render(f"{weed.id}", True, BLACK)
            #self.screen.blit(id_text, (pixel_x + 10, pixel_y - 10))

        # Optional: Draw weeds from weed_list (similar logic as above)
        for weed in self.plant_positions:
            # Convert weed coordinates to pixels
            pixel_x = int(weed[0] / self._FRAME_DISCRETIZATION * self._CELL_WIDTH)
            pixel_y = int(weed[1] / self._FRAME_DISCRETIZATION * self._CELL_HEIGHT)
            pygame.draw.circle(self.screen, BLACK, (pixel_x, pixel_y), 5)

        # Draw y-axis numbering (right side)
        for i in range(int(self._FRAME_HEIGHT / self._FRAME_DISCRETIZATION)):
            label = f"{i * self._FRAME_DISCRETIZATION:.1f}"
            text = self.font.render(label, True, BLACK)
            self.screen.blit(text, (self._SCREEN_WIDTH + 10, i * self._CELL_HEIGHT - self._FONT_SIZE // 2))

        # Draw x-axis numbering (bottom)
        # Integer part (e.g., 00000)
        for i in range(int(self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)):
            x_pos = i * self._CELL_WIDTH
            label = f"{int(i * self._FRAME_DISCRETIZATION // 1)}"
            text = self.font.render(label, True, BLACK)
            self.screen.blit(text, (x_pos + self._CELL_WIDTH // 4, self._SCREEN_HEIGHT + 5))

        # Dots row (e.g., . . . . .)
        for i in range(int(self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)):
            x_pos = i * self._CELL_WIDTH
            dot_text = self.font.render(".", True, BLACK)
            self.screen.blit(dot_text, (x_pos + self._CELL_WIDTH // 4, self._SCREEN_HEIGHT + 20))

        # Fractional part row (e.g., 0123456789)
        for i in range(int(self._FRAME_WIDTH / self._FRAME_DISCRETIZATION)):
            x_pos = i * self._CELL_WIDTH
            label = f"{int(i * self._FRAME_DISCRETIZATION % 1 * 10)}"
            text = self.font.render(label, True, BLACK)
            self.screen.blit(text, (x_pos + self._CELL_WIDTH // 4, self._SCREEN_HEIGHT + 35))

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
