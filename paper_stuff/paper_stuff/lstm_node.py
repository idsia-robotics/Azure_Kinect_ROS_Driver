from .model_node import LSTMNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = LSTMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()