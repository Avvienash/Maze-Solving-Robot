from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
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
import threading

# Define the size of the image (n x n)
image_size = 200  # Replace this with the actual size of your image
roi = (10, 22, 182, 200) 



# Define Global Variables
ACTION = 'NONE'
STATUS = 'Setting Up...'
X_POS  = -1
Y_POS = -1
IMAGE = np.ones((image_size, image_size, 3), np.uint8) * 255


app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html', status = STATUS)


def gen_frames():
    global IMAGE

    while True:
        # Capture frame-by-frame
        frame =IMAGE.copy()

        if frame is not None:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Yield the frame in byte format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_coordinates', methods=['POST'])
def get_coordinates():
    global  X_POS, Y_POS
    # Get the coordinates from the POST request
    x = float(request.form['x'])
    y = float(request.form['y'])
    

    # Calculate the corresponding (x, y) coordinate in the NumPy array
    # Assuming the image is square
    if STATUS == "Please Selected a Goal Position" :
        
        X_POS = int(x * (roi[2]-roi[0]) + roi[0])
        Y_POS = int(y * (roi[3]-roi[1]))
        
        # X_POS = int(x * (200))
        # Y_POS = int(y * (200))
        print(f'X: {X_POS}, Y: {Y_POS}')
    
    return  jsonify({'x': x, 'y': y})

@app.route('/get_status', methods=['GET'])
def get_status():
    global STATUS
    return jsonify({'status': STATUS})

@app.route('/update_action', methods=['POST'])
def update_action():
    global ACTION, IMAGE, STATUS
    
    action = request.form['action']
    ACTION = action
    
    print('Action: ', ACTION)
    return jsonify({'message': 'Action updated successfully'})



def threaded_function():
    global STATUS, IMAGE, ACTION, X_POS, Y_POS
    
    spi = spi_setup()
    picam2 = cam_setup()
    send_data(spi, 1, 0, 1, 0)
    time.sleep(5)

    
    while True:
        
        STATUS = "Please Selected a Goal Position" 
        (x,y), image = get_ball_position(picam2, save_image = False)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        IMAGE =  image
        
        if X_POS != -1 and Y_POS != -1:
            
            if x == -1:
                STATUS = 'NO BALL'
                time.sleep(5)
                continue
                            
            STATUS = "Goal Position Selected, Planning Path"
            target = (X_POS,Y_POS)
            print(f"Target: {target}")
            cv2.circle(image, target, 3, (0, 255, 0), -1)
            IMAGE = image
            X_POS = -1
            Y_POS = -1
            (x,y), image = get_ball_position(picam2, save_image = False)
            occupancy_map = get_map(picam2, save_image = False)
            occupancy_map = remove_obstacles_within_radius(occupancy_map, (x,y), radius = 9)
            rrt_star_nodes, path = generate_rrt_star(occupancy_map, (y,x), (target[1],target[0]), max_iters=20000, step_size=5, sample_rate=0.11, radius=7, stop_when_reached=False)

            print("Plotting Path")
            STATUS = 'Plotting Path'
            print(path)
            image, overlay = plot_rrt_star(occupancy_map, image, rrt_star_nodes, path, (y,x), (target[1],target[0]))
            if path == []:
                STATUS = 'Path not Found'
                time.sleep(5)
                continue
            STATUS = 'Path Found, Press Solve to begin'
            IMAGE = image
            
            path = [(y, x) for x, y in path]
            print(path)
            print(len(path))
            
            
            while ACTION != 'solve':
                if ACTION == 'cancel_run':
                    break
            
            if ACTION == 'cancel_run':
                ACTION = 'NONE'
                continue
            elif ACTION == 'solve':
                
                ACTION = 'NONE'
                STATUS = 'Solving Path'
                print("Solving Path")
                
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
                targets_x = [target[0]]
                targets_y = [target[1]]
                x_pos = []
                y_pos = []
                dt = []
                
                # Waypoint counter
                i = 0
                reached_flag = False
                time_now = datetime.now()
                
                while ACTION != 'cancel_run':
                    (x,y), image = get_ball_position(picam2, save_image = False)
                    image = cv2.add(image, overlay)
                    image = cv2.circle(image, target, 3, (0, 255, 0), -1)
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    IMAGE = image
                    
                    if (x < 0):
                        STATUS = 'Solving Path, No Ball'
                        print("No Ball")
                        output_x = output_x - 1*(np.sign(output_x))
                        output_y = output_y - 1*(np.sign(output_y))
                        output_x = max(min(int(output_x), 50), -50) * -1
                        output_y = max(min(int(output_y), 50), -50) * -1
                        send_data(spi, int(output_y > 0), abs(output_y),  int(output_x > 0), abs(output_x))
                        time.sleep(0.3)
                        continue
                    
                    if not reached_flag:
                        STATUS = 'Solving Path'
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
                                STATUS = 'Solving Path, Recalculating Route'
                                path = generate_new_path((int(y), int(x)),nodes = rrt_star_nodes, step_size = 5 ,radius = 7, image=image)
                                if len(path) == 0:
                                    continue
                                print("Path :", path)
                                pidx.setPoint(path[0][0])
                                pidy.setPoint(path[0][1])
                                i = 0
                            
                            
                    else:
                        print("Reached Goal")
                        STATUS = 'Reached Goal'
                        if (abs(output_x) <= 2 and abs(output_y) <= 2):
                            time.sleep(2)
                            while (abs(output_x) != 0 and abs(output_y) != 0):
                                output_x = output_x - 1*(np.sign(output_x))
                                output_y = output_y - 1*(np.sign(output_y))
                                (x,y), image = get_ball_position(picam2, save_image = False)
                                image = cv2.add(image, overlay)
                                image = cv2.circle(image, target, 3, (0, 255, 0), -1)
                                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                                IMAGE = image
                            break
                        
                        STATUS = 'Reached Goal, Press Cancel to Restart'
                        while ACTION != 'cancel_run':
                            pass
                        ACTION = 'NONE'
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
            
                
                
                

    
if __name__ == "__main__":
    try:
        thread = threading.Thread(target=threaded_function)
        thread.start()
        
        app.run(debug=False)
        
    except KeyboardInterrupt:
        pass
    
    thread.join()
