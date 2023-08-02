import matplotlib.pyplot as plt
import numpy as np
import time


def reset():

    orders =np.array([])
    n_orders=4
    obstacles = []
    square_used = [(0,0)]

    orders = place_orders(n_orders)
    square_used += orders
    #print(square_used)
    obstacles = place_obstacle(square_used)
    square_used, carrer = place_carrer(-2,2,-2,2,0,0,square_used)
    #print(square_used)
    obstacles=carrer

    square_used, horiz = place_horiz(-2,2,-2,2,0,0,square_used)
    #print(square_used)
    obstacles+=horiz
    square_used, casse = place_casse(-2,2,-2,2,0,0,square_used)
    obstacles+=casse

    obs_x = [x for x, y in obstacles]
    obs_y = [y for x, y in obstacles]
    o_x = [x for x, y in orders]
    o_y = [y for x, y in orders]
    everything(o_x,o_y,obs_x,obs_y)
def place_obstacle(square_used) :
def place_orders(n_orders) : 
    orders =[]
    while len(orders) != n_orders:
                
                orders.append((__receive_order(-2,2,-2,2,0,0)))
                #print(orders)
               
    print("order placed")
    return orders

def place_carrer(map_min_x=-2,map_max_x=2,map_min_y=-2,map_max_y=2,restaurant_x=0,restaurant_y=0,square_use=[0,0]):

    
    carrer=True
    flag=True
    i=0

    while carrer :
        i=i+1
        #print(i)
        flag=True
        obstacle_x = np.random.choice([i for i in range(map_min_x, map_max_x ) if i != restaurant_x ], 1)[0]
        obstacle_y = np.random.choice([i for i in range(map_min_y+1, map_max_y+1 ) if i != restaurant_y ], 1)[0]    
        
        placement= [(obstacle_x,obstacle_y),(obstacle_x,obstacle_y-1),(obstacle_x+1,obstacle_y),(obstacle_x+1,obstacle_y-1)]
        #print("placement",placement)
        #print("square_use",square_use)
        for i in range(0,4) :
            for j in range(0,len(square_use)) :

                #print(placement[i][0] ,'==', square_use[j][0] ,'and', placement[i][1] ,'==', square_use[j][1] )
                if placement[i][0] == square_use[j][0] and placement[i][1] == square_use[j][1] : 
                    flag=False
        if flag :
            carrer=False
        if i >= 10000 :
            carrer=False
            print("----------------------------SQUARE fail----------------------------")
            placement=[(7,7)]
    print("Carrer")
    square_use+= placement
    square_hitobx = [(obstacle_x-1,obstacle_y+1),(obstacle_x,obstacle_y+1),(obstacle_x+1,obstacle_y+1),(obstacle_x+2,obstacle_y+1),
                     (obstacle_x-1,obstacle_y  ),                                                      (obstacle_x+2,obstacle_y  ),
                     (obstacle_x-1,obstacle_y-1),                                                      (obstacle_x+2,obstacle_y-1),
                     (obstacle_x-1,obstacle_y-2),(obstacle_x,obstacle_y-2),(obstacle_x+1,obstacle_y-2),(obstacle_x+2,obstacle_y-2)]
            
    square_use+=square_hitobx
    return square_use, placement
   
def place_horiz(map_min_x=-2,map_max_x=2,map_min_y=-2,map_max_y=2,restaurant_x=0,restaurant_y=0,square_use=[0,0]):
    flag=True
    horiz=True
    i=0
    while horiz :
        i=i+1        
        flag=True
        obstacle_x = np.random.choice([i for i in range(map_min_x, map_max_x+1 ) if i != restaurant_x], 1)[0]
        obstacle_y = np.random.choice([i for i in range(map_min_y+1, map_max_y+1 ) if i != restaurant_y ], 1)[0]
        
        placement= [(obstacle_x,obstacle_y),(obstacle_x,obstacle_y-1)]

        for i in range(0,2) :
            for j in range(0,len(square_use)) :

                #print(placement[i][0] ,'==', square_use[j][0] ,'and', placement[i][1] ,'==', square_use[j][1] )
                if placement[i][0] == square_use[j][0] and placement[i][1] == square_use[j][1] : 
                    flag=False
        
        if i >= 10000 :
            horiz=False
            print("---------------------------STICK fail------------------------------")
            placement=[(7,7)]
        if flag :
            horiz=False

    print("stick") 

      
    square_use+= placement
    square_stick = [(obstacle_x-1,obstacle_y+1),(obstacle_x,obstacle_y+1), (obstacle_x+1,obstacle_y+1),
                     (obstacle_x-1,obstacle_y  ),                          (obstacle_x+1,obstacle_y  ),
                     (obstacle_x-1,obstacle_y-1),                          (obstacle_x+1,obstacle_y-1),
                     (obstacle_x-1,obstacle_y-2),(obstacle_x,obstacle_y-2),(obstacle_x+1,obstacle_y-2)]    
    square_use+=square_stick
    return square_use, placement
            
def place_casse(map_min_x=-2,map_max_x=2,map_min_y=-2,map_max_y=2,restaurant_x=0,restaurant_y=0,square_use=[0,0]):        
    flag=True
    casse=True
    i=0
    while casse :
        i=i+1

        flag=True
        obstacle_x = np.random.choice([i for i in range(map_min_x, map_max_x+1 ) if i != restaurant_x], 1)[0]
        obstacle_y = np.random.choice([i for i in range(map_min_y, map_max_y+1 ) if i != restaurant_y ], 1)[0]
        
        placement= [(obstacle_x,obstacle_y)]

        
        for j in range(0,len(square_use)) :
            if placement[0][0] == square_use[j][0] and placement[0][1] == square_use[j][1] : 
                flag=False
        if i >= 10000 :
            casse=False
            print("---------------------------Casse fail------------------------------")
            placement=[(7,7)]
        if flag :
            casse=False

    print("casse")       
    square_use+= placement
    return square_use, placement
                   
def __receive_order(map_min_x,map_max_x,map_min_y,map_max_y,restaurant_x,restaurant_y):

    # Generate a single order, not at the center (where the restaurant is)
    order_x = np.random.choice(
        [i for i in range(map_min_x, map_max_x + 1) if i != restaurant_x], 1
    )[0]
    order_y = np.random.choice(
        [i for i in range(map_min_y, map_max_y + 1) if i != restaurant_y], 1
    )[0]

    return order_x, order_y


def create_grid(start, end, step):
    grid = []
    current_value = start
    while current_value <= end:
        grid.append(current_value)
        current_value += step
    return grid
def grille() :
    
    grid=create_grid(-2.5,2.5,1)
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


def everything(o_x,o_y,obs_x,obs_y) :
    grille()

    plt.plot(o_x,o_y,'yo')
    plt.plot(obs_x,obs_y,'rX')

    plt.show()
    time.sleep(0.2)
    plt.close()

for i in range(0, 100) : 
    print("---reset---")
    reset()



