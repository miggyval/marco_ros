from sensor_msgs.msg import JointState
import rospy

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class MarcoJointStatePublisher():
    def __init__(self):
        rospy.init_node('marco_joint_state_publisher', anonymous=True)
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/joints', '')
                                                                
        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})
        
        for controller in sorted(self.joints):
            self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)
                           
        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState)
       
        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
       
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def publish_joint_states(self):
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
           
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        marco = MarcoJointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass