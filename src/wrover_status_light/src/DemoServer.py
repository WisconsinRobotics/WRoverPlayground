import rospy
from wrover_status_light.srv import DoubleService


def double_service_handler(request):
    return request.input * 2

def double_service_server():
    rospy.init_node("double_server")
    s = rospy.Service("double_service", DoubleService, double_service_handler)
    rospy.spin()

if __name__ == "__main__":
    double_service_server()