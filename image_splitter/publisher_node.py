import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSplitterPublisher(Node):
    def __init__(self):
        super().__init__('image_splitter_publisher')

        # This is most likely not the best way.
        # better would be to make multiple id:s not topics.
        self.img_publishers = []
        for i in range(1,8):
            topic_name = f'image_piece_{i}'
            pub = self.create_publisher(Image, topic_name, 10)
            self.img_publishers.append(pub)
        self.bridge = CvBridge()


    # num_rows and num_cols need to be modified to change the number of image pieces
    def split_image(self, image, num_rows=2, num_cols=4):
        pieces = []
        height , width, channels = image.shape
        piece_height = height // num_rows
        piece_width = width // num_cols
        for i in range(num_rows):
            for j in range(num_cols):
                piece = image[i * piece_height:(i+1) * piece_height,
                        j * piece_width:(j + 1) * piece_width]
                pieces.append(piece)
                
        return pieces

    def publish_image_pieces(self, image_path):
    
        image = cv2.imread(image_path)
                   
        pieces = self.split_image(image)
        self.get_logger().info(f"Image split into {len(pieces)} pieces!")
        
        # Saving image to file for debug purposes 
        for i in range(0,len(pieces)):
            cv2.imwrite(f"/home/rlab/imagePart{i}.png", pieces[i])

        for idx, pub in enumerate(self.img_publishers, start=1):
            if idx < len(pieces):
            
                image_piece = pieces[idx]

                msg = self.bridge.cv2_to_imgmsg(image_piece, encoding="bgr8")
                pub.publish(msg)
                self.get_logger().info(f"Published image piece {idx} on topic /image_piece_{idx}.")
                
            else:
                self.get_logger().warn(f"Not enough pieces to publish for index  {idx}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSplitterPublisher()

    image_path = '/home/rlab/img.png'
    node.publish_image_pieces(image_path)
    
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        #node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
