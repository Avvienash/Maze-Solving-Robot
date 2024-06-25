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


def main():
    
    # Setup SPI and Camera
    spi = spi_setup()
    picam2 = cam_setup()
    send_data(spi, 1, 0, 1, 0)
    time.sleep(5)
    
    # Set Target Position
    target_x, target_y = (50,11)
    

    # Define Gains
    Px  = 0.14
    Py = 0.14
    
    Ix = 0.001
    Iy = 0.001
    
    Dx = 0.05
    Dy = 0.05
    

    pidx = PID(Px, Ix, Dx)
    pidy = PID(Py, Iy, Dy)
    
    pidx.setPoint(target_x)
    pidy.setPoint(target_y)

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
                #x = input("Enter to continue")
                continue
            
                
            output_x = pidx.update(x)
            output_y = pidy.update(y)

            output_x = max(min(int(output_x), 30), -30) * -1
            output_y = max(min(int(output_y), 30), -30) * -1
            
            send_data(spi, int(output_y > 0), abs(output_y),  int(output_x > 0), abs(output_x))
            
            print("Output X: ", output_x, "Output Y: ", output_y)
            outputs_x.append(output_x)
            outputs_y.append(output_y)
            targets_x.append(target_x)
            targets_y.append(target_y)
            x_pos.append(x)
            y_pos.append(y)
            dt.append((datetime.now() - time_now).total_seconds())
            
            #time.sleep(0.05)
            time_now = datetime.now()
    
    except KeyboardInterrupt:
        pass
        
            
    print("\n Done")  
    plot_pid_control_signals(outputs_x, outputs_y, y_pos, targets_y, x_pos, targets_x, target_x, target_y, dt)
    print("Done Plotting")
            
    spi.close()
    picam2.stop()

if __name__ == "__main__":
    main()
    