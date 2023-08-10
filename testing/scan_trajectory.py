"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d   
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
from matplotlib import animation
# from drone_3d_trajectory_following import Drone
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from utils.angle import rot_mat_2d
from Mapping.grid_map_lib.grid_map_lib import GridMap

do_animation = True


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 moving_direction, sweep_direction, x_inds_goal_y, goal_y):
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        self.turing_window = []
        self.update_turning_window()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y

    def move_target_grid(self, c_x_index, c_y_index, grid_map):
        n_x_index = self.moving_direction + c_x_index
        n_y_index = c_y_index

        # found safe grid
        if not grid_map.check_occupied_from_xy_index(n_x_index, n_y_index,
                                                     occupied_val=0.5):
            return n_x_index, n_y_index
        else:  # occupied
            next_c_x_index, next_c_y_index = self.find_safe_turning_grid(
                c_x_index, c_y_index, grid_map)
            if (next_c_x_index is None) and (next_c_y_index is None):
                # moving backward
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if grid_map.check_occupied_from_xy_index(next_c_x_index,
                                                         next_c_y_index):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not grid_map.check_occupied_from_xy_index(
                        next_c_x_index + self.moving_direction,
                        next_c_y_index, occupied_val=0.5):
                    next_c_x_index += self.moving_direction
                self.swap_moving_direction()
            return next_c_x_index, next_c_y_index

    def find_safe_turning_grid(self, c_x_index, c_y_index, grid_map):

        for (d_x_ind, d_y_ind) in self.turing_window:

            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not grid_map.check_occupied_from_xy_index(next_x_ind,
                                                         next_y_ind,
                                                         occupied_val=0.5):
                return next_x_ind, next_y_ind

        return None, None

    def is_search_done(self, grid_map):
        for ix in self.x_indexes_goal_y:
            if not grid_map.check_occupied_from_xy_index(ix, self.goal_y,
                                                         occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        # turning window definition
        # robot can move grid based on it.
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")



def find_sweep_direction_and_start_position(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.hypot(dx, dy)

        if d > max_dist:
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]

    return vec, sweep_start_pos


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_position):
    tx = [ix - sweep_start_position[0] for ix in ox]
    ty = [iy - sweep_start_position[1] for iy in oy]
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    converted_xy = np.stack([tx, ty]).T @ rot_mat_2d(th)

    return converted_xy[:, 0], converted_xy[:, 1]


def convert_global_coordinate(x, y, sweep_vec, sweep_start_position):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    converted_xy = np.stack([x, y]).T @ rot_mat_2d(-th)
    rx = [ix + sweep_start_position[0] for ix in converted_xy[:, 0]]
    ry = [iy + sweep_start_position[1] for iy in converted_xy[:, 1]]
    return rx, ry


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    y_index = None
    x_indexes = []

    if from_upper:
        x_range = range(grid_map.height)[::-1]
        y_range = range(grid_map.width)[::-1]
    else:
        x_range = range(grid_map.height)
        y_range = range(grid_map.width)

    for iy in x_range:
        for ix in y_range:
            if not grid_map.check_occupied_from_xy_index(ix, iy):
                y_index = iy
                x_indexes.append(ix)
        if y_index:
            break

    return x_indexes, y_index


def setup_grid_map(ox, oy, resolution, sweep_direction, offset_grid=10):
    width = math.ceil((max(ox) - min(ox)) / resolution) + offset_grid
    height = math.ceil((max(oy) - min(oy)) / resolution) + offset_grid
    center_x = (np.max(ox) + np.min(ox)) / 2.0
    center_y = (np.max(oy) + np.min(oy)) / 2.0

    grid_map = GridMap(width, height, resolution, center_x, center_y)
    grid_map.print_grid_map_info()
    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)
    grid_map.expand_grid()

    x_inds_goal_y = []
    goal_y = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        x_inds_goal_y, goal_y = search_free_grid_index_at_edge_y(
            grid_map, from_upper=False)

    return grid_map, x_inds_goal_y, goal_y


def sweep_path_search(sweep_searcher, grid_map, grid_search_animation=False):
    # search start grid
    c_x_index, c_y_index = sweep_searcher.search_start_grid(grid_map)
    if not grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5):
        print("Cannot find start grid")
        return [], []

    x, y = grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index,
                                                                c_y_index)
    px, py = [x], [y]

    fig, ax = None, None
    if grid_search_animation:
        fig, ax = plt.subplots()
        # for stopping simulation with the esc key.
        fig.canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    while True:
        c_x_index, c_y_index = sweep_searcher.move_target_grid(c_x_index,
                                                               c_y_index,
                                                               grid_map)

        if sweep_searcher.is_search_done(grid_map) or (
                c_x_index is None or c_y_index is None):
            print("Done")
            break

        x, y = grid_map.calc_grid_central_xy_position_from_xy_index(
            c_x_index, c_y_index)

        px.append(x)
        py.append(y)

        grid_map.set_value_from_xy_index(c_x_index, c_y_index, 0.5)

        if grid_search_animation:
            grid_map.plot_grid_map(ax=ax)
            plt.pause(1.0)

    return px, py


def planning(ox, oy, resolution,
             moving_direction=SweepSearcher.MovingDirection.RIGHT,
             sweeping_direction=SweepSearcher.SweepDirection.UP,
             ):
    sweep_vec, sweep_start_position = find_sweep_direction_and_start_position(
        ox, oy)
    #print("sweep_start_position",sweep_start_position)
    #print("sweep_vec",sweep_vec)
    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec,
                                       sweep_start_position)

    grid_map, x_inds_goal_y, goal_y = setup_grid_map(rox, roy, resolution,
                                                     sweeping_direction)
    #print("x_inds_goal_y",x_inds_goal_y)
    #print("goal_y",goal_y)
    #print("grid_map",grid_map)
    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,
                                   x_inds_goal_y, goal_y)
    print("sweep_searcher",sweep_searcher)
    px, py = sweep_path_search(sweep_searcher, grid_map)
 
 
    rx, ry = convert_global_coordinate(px, py, sweep_vec,
                                       sweep_start_position)
    #print("rx",rx)
    #print("px",px) 
    #print("ry",ry)
    #print("py",py)

    print("Path length:", len(rx))

    return rx, ry









def planning_animation2(ox, oy, resolution):  # pragma: no cover
    px, py = planning(ox, oy, resolution)

    # animation
    if do_animation:
        for ipx, ipy in zip(px, py):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ox, oy, "-xb")
            plt.plot(px, py, "-r")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)

        plt.cla()
        plt.plot(ox, oy, "-xb")
        plt.plot(px, py, "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)
        plt.close()



def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])



def planning_animation3(ox, oy,oz, resolution):  # pragma: no cover
    px, py = planning(ox, oy, resolution)

    N = len(px)

    
    Z=np.empty(len(px))
    Z.fill(oz[0])

    #print("Z",Z)
    #print("px",px)
    #print("Z",len(Z))
    #print("px",len(px))
    
    
   

   
    data=np.vstack((px,py,Z))
   

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot3D(ox, oy, oz, "-xb")
    ax.axis("equal")
    ax.grid(True)
    
    
    ax.plot3D(px, py, Z, "-r")
    
    for I in range(0,len(px)):
   
        line, = ax.plot(data[0, I:1], data[1, I:1], data[2, I:1])

    
    ax.set_xlabel('X')

    
    ax.set_ylabel('Y')


    ax.set_zlabel('Z')
    
    ani = animation.FuncAnimation(fig, update, N, fargs=(data,line), interval=1, blit=False)
    #ani.save('matplot003.gif', writer='imagemagick')
    plt.show()

def planning_animation33(ox, oy,oz, resolution):  # pragma: no cover
    px, py = planning(ox, oy, resolution)

    
    ax = plt.axes(projection='3d')
    ax.scatter3D(ox, oy,oz,"-xb")

    Z=np.ones([1,len(px)])
   
    Z=Z*oz[0]

    
         
    if do_animation:
        for ipx in zip(px,py,Z):
            ax.cla()
      
            ax.plot3D(ox, oy, oz, "-xb")
            #ax.scatter3D(px, py, Z,"-r")
            ax.scatter3D(px, py, Z, "-xb")
            ax.axis("equal")
            ax.grid(True)
            

        
        ax.plot3D(ox, oy, oz, "-xb")
        ax.scatter3D(px, py, Z,"-r")
        ax.axis("equal")
        ax.grid(True)


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    return ang + 360 if ang < 0 else ang


class Plannification():

    def __init__(self):
        pass

    def planning(self, ox, oy, resolution,
             moving_direction=SweepSearcher.MovingDirection.RIGHT,
             sweeping_direction=SweepSearcher.SweepDirection.UP,
             ):
        sweep_vec, sweep_start_position = find_sweep_direction_and_start_position(
            ox, oy)
        #print("sweep_start_position",sweep_start_position)
        #print("sweep_vec",sweep_vec)
        rox, roy = convert_grid_coordinate(ox, oy, sweep_vec,
                                        sweep_start_position)

        grid_map, x_inds_goal_y, goal_y = setup_grid_map(rox, roy, resolution,
                                                        sweeping_direction)
        #print("x_inds_goal_y",x_inds_goal_y)
        #print("goal_y",goal_y)
        #print("grid_map",grid_map)
        sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,
                                    x_inds_goal_y, goal_y)
        print("sweep_searcher",sweep_searcher)
        px, py = sweep_path_search(sweep_searcher, grid_map)
    
    
        rx, ry = convert_global_coordinate(px, py, sweep_vec,
                                        sweep_start_position)
        #print("rx",rx)
        #print("px",px) 
        #print("ry",ry)
        #print("py",py)

        print("Path length:", len(rx))

        ax=[]
        ay=[]
        

            
        ax.append(rx[0])
        ay.append(ry[0])
        
        for i in range(1, len(rx)-1):
        
            a=[rx[i-1],ry[i-1]]
            b=[rx[i],ry[i]]
            c=[rx[i+1],ry[i+1]]
            ang = getAngle(a, b, c)
        
            if ang != 180.0 :
                ax.append(rx[i])
                ay.append(ry[i])
                
        
        ax.append(rx[len(rx)-1])
        ay.append(ry[len(rx)-1])

        return ax, ay
        # return rx, ry


def main():  # pragma: no cover
    print("start!!")

    # ox = [0.0, 50.0, 100, 100.0, 130.0, 40.0, 0.0]
    # oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]

    ox = [0.0, 50.0, 50.0, 0.0, 0.0]
    oy = [0.0, 0.0, 30.0, 30.0, 0.0]
    oz = [5]
    resolution = 2

    #planning_animation3(ox, oy, Z, resolution)

    #px, py= planning(ox, oy, resolution)
    #Z=np.empty(len(px))
    #Z.fill(oz[0])

    #Drone(px, py, Z, ox, oy, oz)






   

    
    #planning_animation2(ox, oy, resolution)

   # ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
    #oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    #oz = [20]
    #resolution = 5.0
    #planning_animation3(ox, oy, oz, resolution)

    #if do_animation:
     #   plt.show()
    #print("done!!")

    px, py= planning(ox, oy, resolution)

    print("px",px)
    print("py",py)

    plt.plot(px, py)
    plt.show()

    # ix=np.empty(len(ox))
    # ix.fill(oz[0])
    # oz=ix
    # Z=np.empty(len(px))
    # Z.fill(oz[0])
    # Drone(px, py, Z, ox, oy, oz)

if __name__ == '__main__':
    main()