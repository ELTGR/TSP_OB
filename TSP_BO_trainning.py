import os


from stable_baselines3 import PPO
from stable_baselines3.td3.policies import MlpPolicy

from TSP_BO import TSPEasyBatteryEnv

# Set the parameters for the implementation
MAX_TIMESTEPS = 100000000000000000  # Maximum number of steps to perform
SAVE_TIMESTEPS = 10000000 # save model every SAVE_TIMESTEPS step
models_dir = "models"
logs_dir = "logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logs_dir):
    os.makedirs(logs_dir) 

env = TSPEasyBatteryEnv()
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logs_dir, device="cpu")

iteration = 0
while SAVE_TIMESTEPS * iteration < MAX_TIMESTEPS:
    iteration += 1
    model.learn(total_timesteps=SAVE_TIMESTEPS, reset_num_timesteps=False)
    model.save(f"{models_dir}/TSPEasyBatteryEnv_{SAVE_TIMESTEPS * iteration}")
env.close()