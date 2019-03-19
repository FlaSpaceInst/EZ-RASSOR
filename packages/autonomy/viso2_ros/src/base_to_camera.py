#!/usr/bin/env python  
import rospy    
import tf_conversions
import tf2_msgs.msg
import tf2_ros
import geometry_msgs.msg
     
class base_link_to_camera():
 
    def __init__(self):
        # This will allow us to call the publish easiliy in the while loop
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # This will run the process at 10 Hertz
            rospy.sleep(0.1)
    
            # Make an instance of the transform
            t = geometry_msgs.msg.TransformStamped()

            # This will give the transform a header with the time the transform is taking place, allowing it to be time synced later if needed.        
            t.header.stamp = rospy.Time.now()

            # Header frame is the parent. Child frame is its parent. Transforms are from the parent to the child.
            t.header.frame_id = "/base_link"
            t.child_frame_id = "left_camera_optical_frame"
            
            # This is the change in xzy coordinates between the base frame and the camera
            t.transform.translation.x = 0.75
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
        
            # This represents the change in rotation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.785
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = .619495763
 
            # Publish the transform to /tf
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm) 
   
if __name__ == '__main__':
    
    rospy.init_node('base_link_to_camera')
    tfb = base_link_to_camera()
    rospy.spin()


