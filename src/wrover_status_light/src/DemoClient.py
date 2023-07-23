import rospy
from wrover_status_light.srv import DoubleService
from wrover_status_light.srv import DoubleServiceResponse
from wrover_status_light.srv import DoubleServiceRequest

def main():
    rospy.init_node("service_client")
    double_service = rospy.ServiceProxy("double_service", DoubleService)
    rate = rospy.Rate(10)
    num = 1

    rospy.wait_for_service("double_service")
    while not rospy.is_shutdown():
        print(num)
        response:DoubleServiceResponse = double_service(num)
        num = response.output
        rate.sleep()

if __name__ == "__main__":
    main()