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
from spi import *
from cam import *
from PID import *
from scipy.ndimage import binary_dilation


def balance():
    
    # Setup SPI and Camera
    spi = spi_setup()
    picam2 = cam_setup()
    send_data(spi, 1, 0, 1, 0)
    time.sleep(5)
    
    # Set Target Position
    #target_x, target_y = (168,158)
    #target_x, target_y = (170,158)
    target_x, target_y = (93,88) # m3
    
    # Get Path
    path, rrt_star_nodes, image = get_path(picam2, (int(target_y), int(target_x)), max_iters=40000, step_size=5, sample_rate=0.11, radius=7, stop_when_reached=True)
    
    # Check if Path exist
    if len(path) == 0:
        print("No Path Found")
        return
    x = input("Enter to continue")

    
    # Define Gains
    Px  = 0.23
    Py = 0.23
    
    Ix = 0.005
    Iy = 0.005
    
    Dx = 0.003
    Dy = 0.003

    
    pidx = PID(Px, Ix, Dx)
    pidy = PID(Py, Iy, Dy)
    
    pidx.setPoint(path[0][0])
    pidy.setPoint(path[0][1])

    # Lists to store PID outputs and time steps
    outputs_x = []
    outputs_y = []
    targets_x = [target_x]
    targets_y = [target_y]
    x_pos = []
    y_pos = []
    dt = []
    
    # Waypoint counter
    i = 0
    reached_flag = False
    try:
        time_now = datetime.now()
        while True:
            (x,y), _ = get_ball_position(picam2, save_image = False)
            
            if (x < 0):
                print("No Ball")
                output_x = output_x - 1*(np.sign(output_x))
                output_y = output_y - 1*(np.sign(output_y))
                output_x = max(min(int(output_x), 50), -50) * -1
                output_y = max(min(int(output_y), 50), -50) * -1
                send_data(spi, int(output_y > 0), abs(output_y),  int(output_x > 0), abs(output_x))
                time.sleep(0.3)
                continue
            
            if not reached_flag:
                dist = math.sqrt((x - path[i][0])**2 + (y - path[i][1])**2)
                print("waypoint:", i, ":", path[i])
                print("Distance:", dist)  
                if dist < 10: #9
                    if i + 1 < len(path):
                        i += 1
                        print("Reached Waypoint:", i)
                        pidx.setPoint(path[i][0])
                        pidy.setPoint(path[i][1])
                    if i == len(path) - 1:
                        reached_flag = True
                elif dist > 25:
                    
                    min_dist = np.inf
                    for j in range(len(path)):
                        waypoint = path[j]
                        waypoint_dist = math.sqrt((x - waypoint[0])**2 + (y - waypoint[1])**2)
                        if (waypoint_dist < 5):
                            min_dist = waypoint_dist
                            min_index = j
                            
                    if min_dist < 10:
                        i = min_index
                        pidx.setPoint(path[i][0])
                        pidy.setPoint(path[i][1])
                    
                    else:
                        print("Recalculating Route")
                        path = generate_new_path((int(y), int(x)),nodes = rrt_star_nodes, step_size = 5 ,radius = 7, image=image)
                        if len(path) == 0:
                            continue
                        print("Path :", path)
                        pidx.setPoint(path[0][0])
                        pidy.setPoint(path[0][1])
                        i = 0
                    
                    
            else:
                print("Reached Goal")
                if (abs(output_x) <= 2 and abs(output_y) <= 2):
                    time.sleep(2)
                    while (abs(output_x) != 0 and abs(output_y) != 0):
                        output_x = output_x - 1*(np.sign(output_x))
                        output_y = output_y - 1*(np.sign(output_y))
                        time.sleep(0.3)
                    break
                
            output_x = pidx.update(x)
            output_y = pidy.update(y)

            output_x = max(min(int(output_x), 50), -50) * -1
            output_y = max(min(int(output_y), 50), -50) * -1
            
            send_data(spi, int(output_y > 0), abs(output_y),  int(output_x > 0), abs(output_x))
            
            print("Output X: ", output_x, "Output Y: ", output_y)
            
            outputs_x.append(output_x)
            outputs_y.append(output_y)
            targets_x.append(path[i][0])
            targets_y.append(path[i][1])
            x_pos.append(x)
            y_pos.append(y)
            dt.append((datetime.now() - time_now).total_seconds())
            
            #time.sleep(0.05)
            time_now = datetime.now()
    
    except KeyboardInterrupt:
        pass
        
            
    print("Done")  
    plot_pid_control_signals(outputs_x, outputs_y, y_pos, targets_y, x_pos, targets_x, target_x, target_y, dt)
            
    spi.close()
    picam2.stop()

if __name__ == "__main__":
    balance()
    