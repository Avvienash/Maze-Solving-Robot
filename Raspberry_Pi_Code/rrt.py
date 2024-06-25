import numpy as np
import matplotlib.pyplot as plt
import random
import math
from tqdm import tqdm
import cv2


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        
def create_circular_mask(shape, center, radius):
    Y, X = np.ogrid[:shape[0], :shape[1]]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
    mask = dist_from_center <= radius
    return mask

def remove_obstacles_within_radius(occupancy_map, start_position, radius):
    mask = create_circular_mask(occupancy_map.shape, start_position, radius)
    occupancy_map[mask] = 0  # Remove obstacles within the circular region
    return occupancy_map

def generate_rrt_star(occupancy_map, start, goal, max_iters=1000, step_size=10, sample_rate=0.1, radius=50, stop_when_reached=False):
        
    reached_flag = False
    start_nodes = []
    map_width = occupancy_map.shape[0]
    map_height = occupancy_map.shape[1]
    
    remove_obstacles_within_radius(occupancy_map, (goal[1],goal[0]), step_size//2)
    
    # Create a list to hold nodes of the RRT
    nodes = [Node(goal[0], goal[1])]

    for iter in tqdm(range(max_iters), desc='Generating RRT*'):
        if random.random() < sample_rate:
            rand_node = Node(start[0], start[1])
        else:
            rand_node = Node(random.randint(0, map_width - 1), random.randint(0, map_height - 1))
        
        # Find the nearest node in the tree to the randomly sampled point
        nearest_node = nodes[0]
        min_dist = math.sqrt((rand_node.x - nearest_node.x) ** 2 + (rand_node.y - nearest_node.y) ** 2)

        for node in nodes:
            dist = math.sqrt((rand_node.x - node.x) ** 2 + (rand_node.y - node.y) ** 2)
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        
        # Check if a step towards the randomly sampled point is feasible
        theta = math.atan2(rand_node.y - nearest_node.y, rand_node.x - nearest_node.x)
        new_x = np.clip(int(nearest_node.x + step_size * math.cos(theta)),0,map_width-1)
        new_y = np.clip(int(nearest_node.y + step_size * math.sin(theta)),0,map_height-1)
        mid_x = int((nearest_node.x + new_x) / 2)
        mid_y = int((nearest_node.y + new_y) / 2)

        if (occupancy_map[new_x, new_y] == 0) and (occupancy_map[mid_x, mid_y] == 0):
            new_node = Node(new_x, new_y)
            new_node.parent = nearest_node

            # Find nearby nodes within a certain radius
            # near_nodes = []
            # for node in nodes:
            #     if math.sqrt((new_node.x - node.x) ** 2 + (new_node.y - node.y) ** 2) < radius:
            #         near_nodes.append(node)
            
            # # Update the parent of the new node if a shorter path is found
            # for node in near_nodes:
            #     if node == nearest_node or occupancy_map[node.x, node.y] == 1:  # Skip invalid nodes
            #         continue
                
            #     temp_cost = node_cost(node) + math.sqrt((new_node.x - node.x) ** 2 + (new_node.y - node.y) ** 2)
            #     if temp_cost < node_cost(new_node):
            #         new_node.parent = node
            
            nodes.append(new_node)
            
            if math.sqrt((new_node.x - start[0]) ** 2 + (new_node.y - start[1]) ** 2) < step_size:
                start_node = Node(start[0], start[1])
                start_node.parent = new_node
                reached_flag = True
                start_nodes.append(start_node)
                nodes.append(start_node)
                
                if stop_when_reached:
                    print("Path Found")
                    break
        
    if not reached_flag:
        print("RRT* could not find a path to the goal")
        return nodes, []
    
    path = generate_path(start_nodes, nodes,radius)
    
    return nodes, path


def generate_new_path(start,nodes,step_size,radius,image):
    start_nodes = []
    
    for node in nodes:
        if math.sqrt((node.x - start[0]) ** 2 + (node.y - start[1]) ** 2) < step_size*3:
            start_nodes.append(node)
    
    if len(start_nodes) == 0:
        print("Goal not found")
        return []
    
    path = generate_path(start_nodes, nodes,radius)
    
    # Plot on Image        
    for i in range(len(path)-1):
        image = cv2.line(image,(path[i][1],path[i][0]), (path[i+1][1],path[i+1][0]), (0,0,255) , 1)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite("images/final_image.jpg", image)
    
    path = [(y, x) for x, y in path]
    return path


def generate_path(start_nodes, nodes,radius):
    
    start_node = start_nodes[0]
    min_cost = node_cost(start_node)
    
    for node in start_nodes:
        if node_cost(node) < min_cost:
            start_node = node
            min_cost = node_cost(node)
    
    print('cost:', min_cost)
    while True:
        path = [(start_node.x, start_node.y)]
        current = start_node
        while current.parent:
            
            near_nodes = []
            for node in nodes:
                if math.sqrt((current.x - node.x) ** 2 + (current.y - node.y) ** 2) < radius:
                    near_nodes.append(node)
            
            for node in near_nodes:
                if node == current.parent:  
                    continue
                
                temp_cost = node_cost(node)
                if temp_cost < node_cost(current.parent):
                    current.parent = node
                    
            path.append((current.parent.x, current.parent.y))
            current = current.parent
            
        new_min_cost = node_cost(start_node)
        print('cost:', new_min_cost)
        if new_min_cost == min_cost:
            print("Path Found")
            break
        else:
            min_cost = new_min_cost

    return path

def generate_path_fast(start_nodes, nodes,radius):
    
    start_node = start_nodes[0]
    min_cost = node_cost(start_node)
    
    for node in start_nodes:
        if node_cost(node) < min_cost:
            start_node = node
            min_cost = node_cost(node)
    
    print('cost:', min_cost)
    path = [(start_node.x, start_node.y)]
    current = start_node
    while current.parent:
        path.append((current.parent.x, current.parent.y))
        current = current.parent
        
    new_min_cost = node_cost(start_node)
    print('cost:', new_min_cost)


    return path
    

def node_cost(node):
    cost = 0
    current = node
    while current.parent:
        cost += math.sqrt((current.x - current.parent.x) ** 2 + (current.y - current.parent.y) ** 2)
        current = current.parent
    return cost

def plot_rrt_star(occupancy_map,image, nodes,path, start, goal):
    
    # Plot on Map
    plt.imshow(occupancy_map, cmap='binary', origin='lower')
    
    for node in nodes:
        if node.parent:
            plt.plot([node.y, node.parent.y], [node.x, node.parent.x], color='blue', alpha=0.5)
            
    for i in range(len(path)-1):
        plt.plot([path[i][1], path[i+1][1]], [path[i][0], path[i+1][0]], color='red')
            
    plt.plot(start[1], start[0],'ro')
    plt.plot(goal[1], goal[0], 'go')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT* Path Planning')
    plt.savefig('images/rrt_star.png')
    plt.clf()
    
    # Plot on Image   
    overlay = np.zeros((image.shape[0], image.shape[1], image.shape[2]), dtype=np.uint8)     
    for i in range(len(path) - 1):
        pt1 = (int(path[i][1]), int(path[i][0]))  # Convert to tuple of integers
        pt2 = (int(path[i+1][1]), int(path[i+1][0]))  # Convert to tuple of integers
        print("pt1: ", pt1)
        print("pt2: ", pt2)
        image = cv2.line(image, pt1, pt2, (0, 0, 255), 1)
        overlay = cv2.line(overlay, pt1, pt2, (0, 0, 255), 1)  # Add alpha channel
        
    cv2.circle(image,(goal[1],goal[0]), 2, (0,0,255), -1)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite("images/final_image.jpg", image)
    
    return image, overlay
    

