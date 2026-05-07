import rospy
import tf
from robosar_messages.srv import *
from robosar_messages.msg import *

def send_homing():
    rospy.init_node('homing', anonymous=True)

    starts = []
    goals = []
    names = []
    agent_active_status = {}

    # Get active agents
    print("calling agent status service")
    rospy.wait_for_service('/robosar_agent_bringup_node/agent_status')
    try:
        get_status = rospy.ServiceProxy('/robosar_agent_bringup_node/agent_status', agent_status)
        resp1 = get_status()
        active_agents = resp1.agents_active
        for a in active_agents:
            agent_active_status[a] = True
        print("{} agents active".format(len(agent_active_status)))
        assert len(agent_active_status) > 0
    except rospy.ServiceException as e:
        print("Agent status service call failed: %s" % e)
        raise Exception("Agent status service call failed")

    # get robot positions
    robot_init=[]
    listener = tf.TransformListener()
    listener.waitForTransform('map', list(agent_active_status.keys())[0] + '/base_link', rospy.Time(), rospy.Duration(1.0))
    for name in agent_active_status:
        now = rospy.Time.now()
        listener.waitForTransform('map', name + '/base_link', now, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('map', name + '/base_link', now)
        robot_init.append([trans[0], trans[1]])

    # fill in message
    for i, name in enumerate(agent_active_status.keys()):
        names.append(name)
        starts.append(robot_init[i])
        if i%2 == 0:
            goals.append([0+0.6*((i+1)//2), 0])
        else:
            goals.append([0 - 0.6 * ((i+1)//2), 0])

    # publish
    task_pub = rospy.Publisher('task_allocation', task_allocation, queue_size=10)
    print("publishing")
    task_msg = task_allocation()
    print(goals)
    task_msg.id = names
    task_msg.startx = [s[0] for s in starts]
    task_msg.starty = [s[1] for s in starts]
    task_msg.goalx = [g[0] for g in goals]
    task_msg.goaly = [g[1] for g in goals]
    while task_pub.get_num_connections() == 0:
         rospy.loginfo("Waiting for subscriber :")
         rospy.sleep(1)
    task_pub.publish(task_msg)
    rospy.sleep(1)
    print("sent")

if __name__ == '__main__':
    try:
        send_homing()
    except rospy.ROSInterruptException:
        pass