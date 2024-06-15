import serial
import matplotlib.pyplot as plt
from collections import deque
from drawnow import drawnow

# Serial port settings
port = 'COM14'  # Replace with your Arduino's port
baud_rate = 115200  # Match this with your Arduino's baud rate

# Data buffer sizes
buffer_size = 100  # Number of points to display

# Initialize data buffers
accX_data = deque(maxlen=buffer_size)
accY_data = deque(maxlen=buffer_size)
accZ_data = deque(maxlen=buffer_size)

# Open serial port
ser = serial.Serial(port, baud_rate)

# Set initial plot limits
y_min = -16384  # Default for ±2g range
y_max = 16384   # Adjust these based on your MPU6050 settings
                # For ±4g: -32768 to 32767

def plot_data():
    plt.clf()  # Clear the current figure
    plt.title('MPU6050 Accelerometer Data')
    plt.xlabel('Sample')
    plt.ylabel('Acceleration (raw)')
    plt.grid()

    if accX_data:
        plt.plot(accX_data, label='AccX', color='red')
    if accY_data:
        plt.plot(accY_data, label='AccY', color='green')
    if accZ_data:
        plt.plot(accZ_data, label='AccZ', color='blue')
    
    plt.legend()
    
    # Dynamically adjust y-axis limits if we have data
    global y_min, y_max
    if accX_data and accY_data and accZ_data:
        current_min = min(min(accX_data), min(accY_data), min(accZ_data))
        current_max = max(max(accX_data), max(accY_data), max(accZ_data))
        
        # Update only if new data expands the range
        y_min = min(y_min, current_min)
        y_max = max(y_max, current_max)
    
    # Add a 10% margin to the y-axis
    margin = (y_max - y_min) * 0.1
    plt.ylim(y_min - margin, y_max + margin)

    # Set x-axis limit to show the entire buffer
    plt.xlim(0, buffer_size)

plt.ion()  # Turn on interactive mode
plt.figure(figsize=(12, 6))  # Set figure size

data_available = False

while True:
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            accX, accY, accZ = map(int, line.split(','))
            
            accX_data.append(accX)
            accY_data.append(accY)
            accZ_data.append(accZ)
            
            data_available = True
        except:
            # Handle any parsing errors
            pass
    
    if data_available:
        drawnow(plot_data)  # Update the plot
    
    plt.pause(0.01)  # Short pause to allow the plot to update and keep the program responsive

# Close serial port when done (you'll need to add a way to break the loop, like a keyboard interrupt)
ser.close()