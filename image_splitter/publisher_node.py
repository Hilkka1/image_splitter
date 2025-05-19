import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_messages.msg import ImagePiece
import sys
import os
import time

class ImageSplitterPublisher(Node):
    def __init__(self, image_directory_path):
        super().__init__('image_splitter_publisher')
        self.image_directory_path = image_directory_path

        """
        # This is most likely not the best way.
        # better would be to make multiple id:s not topics.
        self.img_publishers = []
        for i in range(1,8):
            topic_name = f'image_piece_{i}'
            pub = self.create_publisher(Image, topic_name, 10)
            self.img_publishers.append(pub)
        self.bridge = CvBridge()"""
        topic_name = "image_pieces"

        self.publisher = self.create_publisher(ImagePiece, topic_name, 10)
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0

    # num_rows and num_cols need to be modified to change the number of image pieces
    def split_image(self, image, num_rows=4, num_cols=2):
        pieces = []
        proj_image_width = 1680
        proj_image_height = 1050
        # resize the image to fit the projector aspect ratio
        #image = cv2.resize(image, (proj_image_width, proj_image_height))
        height , width, channels = image.shape
        
        # add the coordinate lines to the image
        red = (0,0,255)
        green = (0,255,0)
        
        # vertical red line
        image = cv2.line(image, (width//2, 0), (width//2, height), red,3)
        # horizontal red line
        image = cv2.line(image, (0,height//2), (width , height//2), red,3)
        
        # vertical green lines
        image = cv2.line(image, (width//4, 0), (width//4, height), green,2)
        image = cv2.line(image, (width//4*3, 0), (width//4*3, height), green,2)
        # horizontal green lines
        image = cv2.line(image, (0,height//4), (width , height//4), green,2)
        image = cv2.line(image, (0,height//4*3), (width , height//4*3), green,3)
        
        
        cv2.imwrite(f"/home/rlab/imagered.png", image)
        
        
        
        piece_height = height // num_rows
        piece_width = width // num_cols
        for i in range(num_rows):
            for j in range(num_cols):
                piece = image[i * piece_height:(i+1) * piece_height,
                        j * piece_width:(j + 1) * piece_width]
                piece = cv2.resize(piece,(proj_image_width, proj_image_height))
                pieces.append(piece)
                
        return pieces
    
    def split_video(self, video_path, num_rows=4, num_cols=2):
    
        video_pieces = []
        indice_of_piece = 0
        
        video = cv2.VideoCapture(video_path)
        # getting the video stats
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(video.get(cv2.CAP_PROP_FPS))
        
        video_piece_height = height//num_rows
        video_piece_width = width//num_cols
        
     
        # setting up the output video
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        
        for i in range(num_rows):
            for j in range(num_cols):
                while True:
            
                    output_video = cv2.VideoWriter(f"output_video{indice_of_piece}.mp4",fourcc, fps, (video_piece_width,video_piece_height))
                
                    ret, frame = video.read()
                    
                    if not ret:
                        break
                
                    cropped_frame = frame[i * video_piece_height:(i+1) * video_piece_height,
                            j * video_piece_width:(j + 1) * video_piece_width]
                    
                    output_video.write(cropped_frame)
                output_video.release()
                indice_of_piece += 1
                
    def add_goal(self, image):
        # ask user for the coordintes for the goal
        
        goal_diameter = 10
        goal_width = 3
        return cv2.circle(image, (self.x,self.y),goal_diameter,(0,0,255),goal_width)
    

    def publish_image_pieces(self, video_path):
        print(self.image_directory_path)
        image_paths = sorted(os.listdir(self.image_directory_path))
        print(image_paths)
        
        
        # Ask user for the time interval as seconds
        time_interval = int(input("Please input the time interval of the image change in seconds: "))
        self.x = int(input("Please add the x-coordinate of the goal as an integer."))
        self.y = int(input("Please add the y-coordinate of the goal as an integer."))
        
        for image_path in image_paths:
                absolute_path = self.image_directory_path + "/" + image_path
                print(image_path)
                image = cv2.imread(absolute_path)
                
                goal_added = self.add_goal(image)
  
                pieces = self.split_image(goal_added)
                

                #self.get_logger().info(f"Image split into {len(pieces)} pieces!")
                
                # Saving image to file for debug purposes 
                #for i in range(0,len(pieces)):
                #    cv2.imwrite(f"/home/rlab/imagePart{i}.png", pieces[i])

                for idx, piece in enumerate(pieces):
                    if idx+1 < len(pieces)+1:
                        msg = ImagePiece()
                        msg.piece_id = idx+1
                        msg.image = self.bridge.cv2_to_imgmsg(piece, encoding="bgr8")
                        self.publisher.publish(msg)
                        
                        #self.get_logger().info(f"Published image piece {idx}.")
                        
                    else:
                        self.get_logger().warn(f"Not enough pieces to publish for index  {idx}")
                # make the system wait until the next image is published.
                time.sleep(time_interval)


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: image_piece_subscriber.py <piece_id>")
        sys.exit(1)

    try:
        image_directory_path = str(sys.argv[1])
    except ValueError:
        print(":(.")
        sys.exit(1)
        
    node = ImageSplitterPublisher(image_directory_path)


    video_path = '/home/rlab/testVideos/videoOfSky.mp4'
    node.publish_image_pieces(video_path)
    
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        #node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
