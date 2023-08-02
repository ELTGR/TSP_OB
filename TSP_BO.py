import gym
import numpy as np
import random
from gym import spaces
#from TSP_view_2D import TSPView2D
import matplotlib.pyplot as plt
import numpy as np
import time


class TSPEasyBatteryEnv(gym.Env):
    #def render(self, mode="human", close=False):

        #if self.tsp_view is None:
            #self.tsp_view = TSPView2D(self.n_orders, self.map_quad, grid_size=25)

        # return self.tsp_view.update(
        #     self.agt_at_restaurant,
        #     self.restaurant_x,
        #     self.restaurant_y,
        #     self.o_delivery,
        #     self.o_x,
        #     self.o_y,
        #     self.agt_x,
        #     self.agt_y,
        #     mode,
        # )

    def __init__(self, n_orders=4, map_quad=(2, 2), max_time=50, randomized_orders=False):

        #self.tsp_view = None
        self.map_quad = map_quad

        self.o_y = []
        self.o_x = []
        
        self.randomized_orders = randomized_orders

        self.n_orders = n_orders
        self.restaurant_x = 0
        self.restaurant_y = 0

        self.agt_x = None
        self.agt_y = None

        self.agt_battery = None
        self.battery_eff = None

        self.o_delivery = []
        self.o_time = []
        self.order_distances = []
        self.agt_at_restaurant = None
        self.agt_time = None
        self.exclusion_square = []
        self.square_used=[(0,0)]
        self.max_time = max_time

        self.map_min_x = -map_quad[0]
        self.map_max_x = +map_quad[0]
        self.map_min_y = -map_quad[1]
        self.map_max_y = +map_quad[1]

        # agent x,
        agt_x_min = [self.map_min_x]
        agt_x_max = [self.map_max_x]
        # agent y,
        agt_y_min = [self.map_min_y]
        agt_y_max = [self.map_max_y]

        # obstacle x,
        # if self.map_max_x == 1 :
        #     coeff = 1 
        # else :
        #     coeff=(self.map_max_x-1)*7
        coeff=1
        obstacle_x_min = [self.map_min_x] * coeff
        obstacle_x_max = [self.map_max_x] * coeff
        # obstacle y,
        obstacle_y_min = [self.map_min_y] * coeff
        obstacle_y_max = [self.map_max_y] * coeff

        #agent battery,
        agt_battery_min = [0]
        agt_battery_max = [100]
        # n_orders for x positions of orders,
        o_x_min = [self.map_min_x for i in range(n_orders)]
        o_x_max = [self.map_max_x for i in range(n_orders)]
        # n_orders for y positions of orders,
        o_y_min = [self.map_min_y for i in range(n_orders)]
        o_y_max = [self.map_max_y for i in range(n_orders)]

        # whether delivered or not, 0 not delivered, 1 delivered
        o_delivery_min = [0] * n_orders
        o_delivery_max = [1] * n_orders

        # distance from restaurant to orders in steps
        order_distances_min = [0] * n_orders
        order_distances_max = [self.map_max_x*2] * n_orders

        # whether agent is at restaurant or not
        agt_at_restaurant_max = 1
        agt_at_restaurant_min = 0

        # Time since orders have been placed
        o_time_min = [0] * n_orders
        o_time_max = [max_time] * n_orders

        # Time since start
        agt_time_min = 0
        agt_time_max = max_time

        self.observation_space = spaces.Box(
            low=np.array(
                agt_x_min #agent pos x
                + agt_y_min #agent pos y
                + o_x_min #goal pos y
                + o_y_min #goal pos y
                + [0] 
                + [0]
                + o_delivery_min
                + [agt_at_restaurant_min]
                + o_time_min
                + [agt_time_min]
                + [0]
                + agt_battery_min
                + order_distances_min
                + obstacle_x_min
                + obstacle_y_min
            ),
            high=np.array(
                agt_x_max
                + agt_y_max
                + o_x_max
                + o_y_max
                + [0]
                + [0]
                + o_delivery_max
                + [agt_at_restaurant_max]
                + o_time_max
                + [agt_time_max]
                + [self.max_time]
                + agt_battery_max
                + order_distances_max
                + obstacle_x_max
                + obstacle_y_max
            ),
            dtype=np.int16,
        )

        # Action space, UP, DOWN, LEFT, RIGHT
        self.action_space = spaces.Discrete(4)

    def reset(self):
        
        self.restaurant_x = 0
        self.restaurant_y = 0
        self.agt_x = self.restaurant_x
        self.agt_y = self.restaurant_y
        self.agt_battery = 100
        self.battery_eff = 1
        self.square_used=[(0,0)]
        self.exclusion_square=[]
        if self.randomized_orders:
            # Enforce uniqueness of orders, to prevent multiple orders being placed on the same points
            # And ensure actual orders in the episode are always == n_orders as expected
            orders = []
            while len(orders) != self.n_orders:
                orders += [self.__receive_order()]
                orders = list(set(orders))
        else:
            orders = [(-2, -2), (1, 1), (2, 0), (0, -2)]
        self.o_x = [x for x, y in orders]
        self.o_y = [y for x, y in orders]
        self.o_delivery = [0] * self.n_orders
        self.o_time = [0] * self.n_orders
        self.agt_at_restaurant = 1
        self.agt_time = 0
        self.order_distances = [abs(x) + abs(y) for x, y in orders]
        self.square_used+=orders
        self.place_exclusion()
        self.obs_x = [x for x, y in self.exclusion_square]
        self.obs_y = [y for x, y in self.exclusion_square]
        
        return self.__compute_state()
    def place_exclusion (self) :
        #if self.map_max_x == 1 :
        self.place_casse()
        # else :
        #     for i in range(0,self.map_max_x-1):
        #         self.place_carrer()
        #         self.place_horiz()
        #         self.place_casse()

    def place_carrer(self):

        if self.randomized_orders :

            carrer=True
            flag=True
            i=0

            while carrer :
                i=i+1
                flag=True
            
                obstacle_x = np.random.choice([i for i in range(self.map_min_x, self.map_max_x ) if i != self.restaurant_x ], 1)[0]
                obstacle_y = np.random.choice([i for i in range(self.map_min_y+1, self.map_max_y+1 ) if i != self.restaurant_y ], 1)[0]    
                placement= [(obstacle_x,obstacle_y),(obstacle_x,obstacle_y-1),(obstacle_x+1,obstacle_y),(obstacle_x+1,obstacle_y-1)]
                
                for i in range(0,4) :
                    for j in range(0,len(self.square_used)) :
                        if placement[i][0] == self.square_used[j][0] and placement[i][1] == self.square_used[j][1] : 
                            flag=False
                if flag :
                    carrer=False

                if i >= 10000 :
                    carrer=False
                    placement=[(7,7)]

        else : 
             obstacle_x=1
             obstacle_y=1
             placement= [(obstacle_x,obstacle_y)]

        square_hitobx = [(obstacle_x-1,obstacle_y+1),(obstacle_x,obstacle_y+1),(obstacle_x+1,obstacle_y+1),(obstacle_x+2,obstacle_y+1),
                         (obstacle_x-1,obstacle_y  ),                                                      (obstacle_x+2,obstacle_y  ),
                         (obstacle_x-1,obstacle_y-1),                                                      (obstacle_x+2,obstacle_y-1),
                         (obstacle_x-1,obstacle_y-2),(obstacle_x,obstacle_y-2),(obstacle_x+1,obstacle_y-2),(obstacle_x+2,obstacle_y-2)]
        self.square_used+= placement
        self.square_used+=square_hitobx
        self.exclusion_square = placement       
               
    def place_horiz(self):

        if self.randomized_orders :

            flag=True
            horiz=True
            i=0

            while horiz :

                i=i+1        
                flag=True
                obstacle_x = np.random.choice([i for i in range(self.map_min_x, self.map_max_x+1 ) if i != self.restaurant_x], 1)[0]
                obstacle_y = np.random.choice([i for i in range(self.map_min_y+1, self.map_max_y+1 ) if i != self.restaurant_y ], 1)[0]
                placement= [(obstacle_x,obstacle_y),(obstacle_x,obstacle_y-1)]

                for i in range(0,2) :
                    for j in range(0,len(self.square_used)) :
                        if placement[i][0] == self.square_used[j][0] and placement[i][1] == self.square_used[j][1] : 
                            flag=False
                
                if i >= 10000 :
                    horiz=False
                    placement=[(7,7)]

                if flag :
                    horiz=False
 
        else : 
             obstacle_x=-1
             obstacle_y=2
             placement= [(obstacle_x,obstacle_y)]
           

        square_stick = [(obstacle_x-1,obstacle_y+1),(obstacle_x,obstacle_y+1),(obstacle_x+1,obstacle_y+1),
                        (obstacle_x-1,obstacle_y  ),                          (obstacle_x+1,obstacle_y  ),
                        (obstacle_x-1,obstacle_y-1),                          (obstacle_x+1,obstacle_y-1),
                        (obstacle_x-1,obstacle_y-2),(obstacle_x,obstacle_y-2),(obstacle_x+1,obstacle_y-2)]
        
        self.square_used+= placement
        self.square_used+=square_stick
        self.exclusion_square += placement
        
    def place_casse(self): 

        if self.randomized_orders :

            flag=True
            casse=True
            i=0
            while casse :

                i=i+1
                flag=True

                obstacle_x = np.random.choice([i for i in range(self.map_min_x, self.map_max_x+1 ) if i != self.restaurant_x], 1)[0]
                obstacle_y = np.random.choice([i for i in range(self.map_min_y, self.map_max_y+1 ) if i != self.restaurant_y ], 1)[0]
                placement= [(obstacle_x,obstacle_y)]

                for j in range(0,len(self.square_used)) :
                    if placement[0][0] == self.square_used[j][0] and placement[0][1] == self.square_used[j][1] : 
                        flag=False

                if i >= 10000 :
                    casse=False
                    placement=[(7,7)]

                if flag :
                    casse=False
        else : 
            obstacle_x=1
            obstacle_y=0
            placement= [(obstacle_x,obstacle_y)]
        square_hitbox = [(obstacle_x-1,obstacle_y+1),(obstacle_x,obstacle_y+1),(obstacle_x+1,obstacle_y+1),
                         (obstacle_x-1,obstacle_y  ),                          (obstacle_x+1,obstacle_y  ),
                         (obstacle_x-1,obstacle_y-1),(obstacle_x,obstacle_y-1),(obstacle_x+1,obstacle_y-1)]
        self.square_used+= square_hitbox
        self.square_used+= placement
        self.exclusion_square += placement

    def everything(self) :

        self.grille()
        
        obs_x = [x for x, y in self.exclusion_square]#[1:]]
        obs_y = [y for x, y in self.exclusion_square]#[1:]]
        
        plt.plot(self.o_x,self.o_y,'ys')
        plt.plot(obs_x,obs_y,'rs')
        plt.plot(self.agt_x,self.agt_x,'bo')

        plt.show()
        

    def grille(self) :
        
        grid=self.create_grid()
        start = -2.5
        end = 2.5
        # Plotting the grid
        plt.figure(figsize=(6, 6))  # Optional: Set the size of the plot

        # Plot vertical lines
        for x in grid:
            plt.axvline(x, color='blue', linestyle='-')

        # Plot horizontal lines
        for y in grid:
            plt.axhline(y, color='blue', linestyle='-')


        

        plt.xlim(start , end )
        plt.ylim(start , end )
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        
    def create_grid(self):
        grid = []
        current_value = self.map_min_x-0.5
        while current_value <= (self.map_max_x+0.5):
            grid.append(current_value)
            current_value += 1
        return grid    
             
    def step(self, action):
        
        done = False
        reward_before_action = self.__compute_reward()
        self.__play_action(action)
        reward = self.__compute_reward() - reward_before_action # - (100 - self.agt_battery)/100

        # If agent completed the route and returned back to start, give additional reward
        if self.agt_at_restaurant:
            done = True
            reward += np.sum(self.o_delivery) * self.max_time - self.__adjust_reward()
            

        # If there is timeout, no additional reward
        if self.agt_time >= self.max_time:
            done = True
        
        if self.agt_battery <= 0 and not self.agt_at_restaurant:
            done = True
            reward -= self.max_time

        info = {}
        #self.everything()
        return self.__compute_state(), reward, done, info

    def __play_action(self, action):
        #save the previous pose 
        agt_pre_xy= (self.agt_x,self.agt_y)
        #look for border 
        if action == 0:  # UP
            self.agt_y = min(self.map_max_y, self.agt_y + 1)
        elif action == 1:  # DOWN
            self.agt_y = max(self.map_min_y, self.agt_y - 1)
        elif action == 2:  # LEFT
            self.agt_x = max(self.map_min_x, self.agt_x - 1)
        elif action == 3:  # RIGHT
            self.agt_x = min(self.map_max_x, self.agt_x + 1)
        else:
            raise Exception("action: {action} is invalid")
        #save the new pose 
        agt_new_xy = (self.agt_x,self.agt_y)
        #look if the new pose is on a exclusion pose
        if agt_new_xy in self.exclusion_square :
                #if yes, new pose = previous pose, cancel the action
                self.agt_x=agt_pre_xy[0]
                self.agt_y=agt_pre_xy[1]

        # Check for deliveries
        for ix in range(self.n_orders):
            if self.o_delivery[ix] == 0:
                if (self.o_x[ix] == self.agt_x) and (self.o_y[ix] == self.agt_y):
                    self.o_delivery[ix] = 1

        # Update the time for the waiting orders
        for ix in range(self.n_orders):
            if self.o_delivery[ix] == 0:
                self.o_time[ix] += 1

        # Update time since agent left restaurant
        self.agt_time += 1

        # Check if agent is at restaurant
        self.agt_at_restaurant = int(
            (self.agt_x == self.restaurant_x) and (self.agt_y == self.restaurant_y)
        )

        #calculate new battery level
        self.agt_battery -= self.battery_eff * 1

    def __compute_state(self):
        
        return (
            [self.agt_x]
            + [self.agt_y]
            + self.o_x
            + self.o_y
            + [self.restaurant_x]
            + [self.restaurant_y]
            + self.o_delivery
            + [self.agt_at_restaurant]
            + self.o_time
            + [self.agt_time]
            + [(self.max_time - self.agt_time)]
            + [self.agt_battery]
            + self.order_distances
            + self.obs_x
            + self.obs_y
        )
        
    def __receive_order(self):

        # Generate a single order, not at the center (where the restaurant is)
        self.order_x = np.random.choice(
            [i for i in range(self.map_min_x, self.map_max_x + 1) if i != self.restaurant_x], 1
        )[0]
        self.order_y = np.random.choice(
            [i for i in range(self.map_min_y, self.map_max_y + 1) if i != self.restaurant_y], 1
        )[0]

        return self.order_x, self.order_y

    def __compute_reward(self):
        return (
            np.sum(np.asarray(self.o_delivery) * self.max_time / (np.asarray(self.o_time) + 0.0001))
            - self.agt_time
        )
    
    def __adjust_reward(self):
        factor = 0
        for i in range(len(self.order_distances)):
            if self.o_delivery[i] == 0:
                if self.order_distances[i] * self.battery_eff < self.agt_battery:
                    factor += 1
        return factor * self.max_time
                   

class TSPMediumBatteryEnv(TSPEasyBatteryEnv):
    def __init__(self, n_orders=4, map_quad=(2, 2), max_time=50, randomized_orders=True):
        super().__init__(n_orders, map_quad, max_time, randomized_orders)
