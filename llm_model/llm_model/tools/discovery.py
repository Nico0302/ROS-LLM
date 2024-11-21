import rospy

class ServiceDiscovery:
    def __init__(self, service_name, service_type):
        self.service_name = service_name
        self.service_type = service_type
        self.service = None
        self.service_found = False

    def service_callback(self, service):
        self.service = service
        self.service_found = True

    def find_service(self):
        rospy.wait_for_service(self.service_name)
        rospy.ServiceProxy(self.service_name, self.service_type)
        rospy.Subscriber(self.service_name, self.service_type, self.service_callback)
        rospy.sleep(1)
        return self