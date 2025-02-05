import rospy
from std_msgs.msg import Float64MultiArray

def RecieveData(msg):
    input_data = msg.data
    for i in range(12):
        output_data[i] = (input_data[-2]+input_data[-3])*10

def SendData():
    send_data = Float64MultiArray(data=output_data)
    OutputDataPublisher.publish(send_data)

if __name__ == "__main__":
    rospy.init_node("mpc")
    input_data = [0.]*46
    output_data = [0.]*12

    InputDataSubscriber = rospy.Subscriber("/simulation_data",Float64MultiArray,RecieveData,queue_size=1)
    OutputDataPublisher = rospy.Publisher("/commands",Float64MultiArray,queue_size=1)

    while not rospy.is_shutdown():
        SendData()
    
    rospy.spin()

