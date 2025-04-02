import json
import socket
import threading
import time
import matplotlib.pyplot as plt
from collections import deque

plt.switch_backend('TkAgg')

UDP_HOST = "0.0.0.0"
UDP_PORT = 12346
BUFFER_SIZE = 1024

MAX_POINTS = 100
time_data = deque(maxlen=MAX_POINTS)
knee_angle_data = deque(maxlen=MAX_POINTS)

gait_data = {"knee_angle": 0.0, "step_time": 0.0, "walking_speed": 0.0}
stop_flag = threading.Event()
sock = None

def udp_listener():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    print(f"Listening on {UDP_HOST}:{UDP_PORT}...")

    while not stop_flag.is_set():
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            message = data.decode()
         
            parsed = json.loads(message)
            if parsed.get("key") == "/gait":
                global gait_data
                gait_data = parsed["value"]
                
        except Exception as e:
            print(f"Error receiving UDP data: {e}")
            time.sleep(0.5)

def update_plot():
    plt.ion()
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    
    ax1.set_title("Knee Angle Over Time", fontsize=14)
    ax1.set_xlabel("Time (seconds)", fontsize=12)
    ax1.set_ylabel("Knee Angle (degrees)", fontsize=12)
    
 
    ax1.set_ylim(0, 180)
    
    
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)


    line1, = ax1.plot(time_data, knee_angle_data, 'b-', label='Knee Angle', linewidth=2)
    ax1.legend(loc="upper right", fontsize=12)
    
    start_time = time.time()
    while not stop_flag.is_set():
        current_time = time.time() - start_time
        time_data.append(current_time)
        knee_angle_data.append(gait_data["knee_angle"])
        
       
        line1.set_xdata(time_data)
        line1.set_ydata(knee_angle_data)
        
       
        ax1.relim()
        ax1.autoscale_view()
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

def cleanup():
    stop_flag.set()
    if sock:
        sock.close()
    print("Cleaning up and exiting...")

if __name__ == "__main__":
    udp_thread = threading.Thread(target=udp_listener)
    udp_thread.start()
    
    try:
        update_plot()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
        cleanup()
        udp_thread.join()
        plt.close('all')
        print("Stopped.")
    except Exception as e:
        print(f"Unexpected error: {e}")
        cleanup()
        udp_thread.join()
        plt.close('all')

