from testing.bluerov_implementation import  ArduSubBluerovImplementation
import rospy
class Bluerov:
    def __init__(self, implementation="ardusub"):
        """
            implementation : choose if you use ArduSub or not ("ardusub" or "simple")
        """

        if implementation == "ardusub":
            
            self.implementation = ArduSubBluerovImplementation()
        else : 
            raise ValueError("Incorrect implementation value. Choose 'ardusub' or 'simple'.")


        try: 
            rospy.init_node('user_mode', log_level=rospy.DEBUG)
            print("INFO : ros node create", 'yellow')

        except rospy.ROSInterruptException as error :
            print('pubs error with ROS: ', error)
            exit(1)

    def move_to(self, coordinates, is_init=False):
        
        self.implementation.move_to(coordinates, is_init)

    def get_current_position(self):
        return self.implementation.get_current_position()
    
    def get_distance_made(self):
        return self.implementation.get_distance_made()

    def get_battery(self):
        return self.implementation.get_battery()
    
    def reset_battery(self):
        return self.implementation.reset_battery()
