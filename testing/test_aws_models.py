import time
from stable_baselines3 import PPO
from stable_baselines3.td3.policies import MlpPolicy
from TSP_BO import  TSPEasyBatteryEnv
from testing.mobile import Bluerov
import rospy
from geometry_msgs.msg import PoseStamped



def publish_goal_position(goal_x, goal_y):
    for number in range(0,4) :
        
        msg = PoseStamped()
      

        msg.pose.position.x = goal_x[number]*10
        msg.pose.position.y = goal_y[number]*10
        msg.pose.position.z = 0
       
        pub = rospy.Publisher('BlueRov2/goal'+str(number+1) ,PoseStamped, queue_size = 1)
        pub.publish(msg)
       # print("pub on BlueRov2/goal"+str(number+1), "postion ", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)


def test_model(model_name,environnement):

    env = environnement()
    model = PPO.load(model_name)
    bluerov_virtuel = Bluerov()
    
    for episode in range(10):
        
        obs = env.reset()
        publish_goal_position(obs[2:6],obs[6:10])
        done = False
        
        bluerov_virtuel.move_to([obs[0],obs[1],0],True)
    
        
        reward_t=0
        while not done:
            
            action, _ = model.predict(obs)
            obs, reward, done, info = env.step(action)
            publish_goal_position(obs[2:6],obs[6:10])   
            

            print("battery :", obs[23])
            reward_t=reward_t+reward
            
            bluerov_virtuel.move_to([obs[0]*10,obs[1]*10,0],False)
            
            time.sleep(0.1)
        print("reward total :", reward_t)

    env.close()

test_model("traveling_salesman_aws_50000000_battery",TSPEasyBatteryEnv)


        

