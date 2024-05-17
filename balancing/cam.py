import spidev
from picamera2 import Picamera2
from datetime import datetime
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math
from rrt import *
from scipy.ndimage import binary_dilation

def cam_setup():
    # Initialize Picamera2
    print("Initializing Camera")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(sensor={'output_size': (900, 900), 'bit_depth': 10},main={"size": (200, 200)} )
    print(config)
    picam2.configure(config)
    picam2.start()
    print("Camera Initialized")
    return picam2

def get_blank_image(picam2):
    np_array = picam2.capture_array()
    roi = (10, 22, 182, 200) 
    return np_array[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2],:]

def get_ball_position(picam2, save_image = False):
    
    np_array = picam2.capture_array()
    roi = (10, 22, 182, 200) 

    # Crop image
    cropped_image = np_array[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2],:]
    
    #channels
    red_channel = cropped_image[:,:,0]  # R channel
    green_channel = cropped_image[:,:,1]  # G channel
    blue_channel = cropped_image[:,:,2]  # B channel


    # Get Ball
    ball_img = red_channel

    #Applying 7x7 Gaussian Blur 
    ball_img_blurred = cv2.GaussianBlur(ball_img, (7, 7), 0)
    
    # Applying threshold 
    ball_threshold = cv2.threshold(ball_img_blurred,120,255,cv2.THRESH_BINARY_INV)[1]
    
    
    # Save Image
    if save_image:
        print("Saving Images")
        cv2.imwrite('images/uncropped_image.jpg', np_array)
        cv2.imwrite('images/cropped_image.jpg', cropped_image)
        cv2.imwrite('images/ball_img.jpg', ball_img)
        cv2.imwrite('images/ball_img_blurred.jpg', ball_img_blurred)
        cv2.imwrite('images/ball_threshold.jpg', ball_threshold)
    
    # Apply the Component analysis function 
    analysis = cv2.connectedComponentsWithStats(ball_threshold,
                                                4, 
                                                cv2.CV_32S) 

    (totalLabels, label_ids, values, centroid) = analysis 

    # Loop through each component 
    for i in range(1, totalLabels): 
        
        # Area of the component 
        area = values[i, cv2.CC_STAT_AREA]  
            
        # Now extract the coordinate points 
        x1 = values[i, cv2.CC_STAT_LEFT] 
        y1 = values[i, cv2.CC_STAT_TOP] 
        w = values[i, cv2.CC_STAT_WIDTH] 
        h = values[i, cv2.CC_STAT_HEIGHT] 
        
        
        if (area > 30) and (w < 40) and (h < 40):
            print("Ball found at x: ", x1 + w/2, " y: ", y1 + h/2, 'h:' , h, 'w:', w)
            cv2.rectangle(cropped_image,(x1, y1),(x1+ w, y1+ h), (0, 0, 0), 1)
            return (x1 + w/2, y1 + h/2), cropped_image
    
    print("No ball found")
    return (-1,-1), cropped_image

def get_map(picam2,save_image = False):
    np_array = picam2.capture_array()
    roi = (10, 22, 182, 200) 

    # Crop image
    cropped_image = np_array[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2],:]

    #channels
    red_channel = cropped_image[:,:,0]  # R channel
    green_channel = cropped_image[:,:,1]  # G channel
    blue_channel = cropped_image[:,:,2]  # B channel

    # Get Walls
    wall_img = green_channel
    wall_img_blurred = cv2.GaussianBlur(wall_img, (7, 7), 0)
    #cv2.imwrite("/home/avvienash/Documents/cam/wall_img_blurred.jpg", wall_img_blurred)

    wall_threshold = cv2.threshold(wall_img_blurred,20,255,cv2.THRESH_BINARY_INV)[1]
    
    structuring_element = np.ones((7,7), dtype=bool)  # You can adjust the size of the structuring element as needed
    inflated_map = binary_dilation((wall_threshold == 255), structure=structuring_element)
    
    if save_image:
        print("Saving Images")
        cv2.imwrite('images/uncropped_image.jpg', np_array)
        cv2.imwrite('images/cropped_image.jpg', cropped_image)
        cv2.imwrite('images/wall_img.jpg', wall_img)
        cv2.imwrite('images/wall_img_blurred.jpg', wall_img_blurred)
        cv2.imwrite('images/wall_threshold.jpg', wall_threshold)
        cv2.imwrite('images/inflated_map.jpg', inflated_map.astype(np.uint8)*255)
        
    return inflated_map

def get_path(picam2,goal, max_iters=100000, step_size=10, sample_rate=0.1, radius=5, stop_when_reached=False):
    
    # Get Map
    occupancy_map = get_map(picam2, save_image = False)
    
    # Get Path
    (x0,y0), image  = get_ball_position(picam2)
    occupancy_map = remove_obstacles_within_radius(occupancy_map, (int(x0),int(y0)), radius)
    print("start:", (x0,y0))

    
    # try:
    print("Generating Path")
    rrt_star_nodes, path = generate_rrt_star(occupancy_map, (int(y0),int(x0)) , goal, max_iters=max_iters, step_size=step_size, sample_rate=sample_rate, radius=radius, stop_when_reached=stop_when_reached)
    
    print("Plotting Path")
    _ = plot_rrt_star(occupancy_map, image, rrt_star_nodes, path, (int(y0),int(x0)), goal)
        
    path = [(y, x) for x, y in path]
    print(path)
    print(len(path))
    
    return path, rrt_star_nodes,  image
    
    # except:
    #     print("No Path Found")
    #     return None, None, image

if __name__ == "__main__":
    
    # Camera Initialisation
    start_time = datetime.now()
    picam2 = cam_setup()
    print("Time taken to initialize camera: ", (datetime.now() - start_time).total_seconds())
    time.sleep(3)
    
    # Get Ball Position
    start_time = datetime.now()
    (x,y), image = get_ball_position(picam2, save_image = True)
    print("Time taken to initialize camera: ", (datetime.now() - start_time).total_seconds())
    
    # Print Labels on Image
    #target_x, target_y = (142,135)
    #target_x, target_y = (90,90) 
    #target_x, target_y = (18,28) 
    #target_x, target_y = (70,41) 
    #target_x, target_y = (168,158) #m1
    target_x, target_y = (168,158) # m2
    #target_x, target_y = (93,88) # m3
    
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.circle(image, (target_x, target_y), 3, (0, 255, 0), -1)
    cv2.imwrite("images/final_image.jpg", image)
    
    # Get Map
    occupancy_map = get_map(picam2, save_image = True)
    
    # # Get Path
    # if x == -1:
    #     print("Ball not found")
    # else:
    #     start_time = datetime.now()
    #     path, rrt_star_nodes, image = get_path(picam2, (int(target_y), int(target_x)), max_iters=50000, step_size=5, sample_rate=0.11, radius=7, stop_when_reached=True)
    #     print("Time taken to get path: ", (datetime.now() - start_time).total_seconds())
    
    # Stop camera
    picam2.stop()