# BotBrains_Battle_Round-2

a) What are the parameters that should be considered while calculating maximum angle of inclination in  Two wheeled self-balancing robots (TWSBR) ?
Answer:
To calculate the maximum angle of inclination for Two-Wheeled Self-Balancing Robots (TWSBR), consider these key parameters:
1.	Center of Mass (CoM) Height: Higher CoM height decreases stability.
2.	Wheelbase (L): Longer wheelbase increases stability.
3.	Wheel Radius (r): Larger wheels generally increase stability.
4.	Inertial Parameters: Mass distribution and moments of inertia affect stability.
5.	Control System: Algorithms and tuning parameters influence balancing capability.
6.	Motor Characteristics: Torque and response time impact balancing performance.
7.	Power Supply: Capacity affects motor torque and stability under load.
8.	Friction Coefficient: Between wheels and ground affects stability.
These factors collectively determine the maximum angle of inclination a TWSBR can achieve while maintaining balance.

b) What is the core concept of the Two wheeled self-balancing robots (TWSBR).
Answer:
The core concept of Two-Wheeled Self-Balancing Robots (TWSBR) is to maintain upright stability using feedback control mechanisms. They dynamically adjust their wheel velocities to counteract any deviation from the upright position, resembling the balancing behavior of an inverted pendulum. This allows them to stay balanced and mobile without tipping over, enabling applications in personal transportation, robotics education, and various autonomous systems.

c) What is the additional thing/component that you can add to make it unique and explain the same by giving its proof of concept.
Answer:
An additional component to make Two-Wheeled Self-Balancing Robots (TWSBR) unique could be a robotic arm for manipulation tasks.
Proof of Concept:
•	Robotic Arm Integration: Add a lightweight robotic arm with grippers to the TWSBR.
•	Task Execution: Demonstrate the robot's ability to pick up and manipulate objects while maintaining balance.
•	Application: Show how the robot can autonomously navigate to a target, grasp an object, and transport it to another location.
This enhancement expands TWSBR capabilities beyond mobility, enabling tasks such as object retrieval and delivery in various environments.


d) Suppose you have been given the computer aided design model of the robot satisfying the necessary clearances and dimensions to hold the parcel. Using the necessary components, you have been assigned the task of programming the robot such that it can carry out the assigned task of delivering the parcel to the consumer doorstep navigating the traffic, turns and other obstacles. Explain how you would achieve this with the code and circuitry to substantiate your claim. It is allowed to take the necessary assumptions if you are certain that it is outside the scope of robotics to obtain the necessary data and then derive conclusions pertaining to that specific aspect.

Answer:
To tackle the task of programming a robot to deliver parcels while navigating through traffic, turns, and obstacles, we need to integrate hardware components effectively and develop robust control algorithms. Here’s a comprehensive approach including theory, calculations, and a sample Python code snippet for controlling the robot.
Hardware Components:
1.	Robot Platform:
o	Two-Wheeled Self-Balancing Robot (TWSBR) designed to carry parcels.
o	Motors with encoders for precise control.
o	IMU (Inertial Measurement Unit) for balance.
o	Camera(s) for vision-based navigation.
o	Distance sensors (ultrasonic or LiDAR) for obstacle detection.
o	Microcontroller (e.g., Arduino, Raspberry Pi) for processing.
2.	Robotic Arm:
o	Lightweight arm with gripper for parcel handling.
3.	Power Management:
o	Battery pack with sufficient capacity for extended operation.
Control Algorithm Development:
1.	Balancing Control:
o	Implement a PID controller to maintain balance using IMU data and motor encoders.
2.	Path Planning and Navigation:
o	Utilize A* algorithm or potential fields method for pathfinding considering obstacles and traffic rules.
3.	Obstacle Avoidance:
o	Integrate sensor data (ultrasonic, LiDAR) to dynamically adjust robot trajectory to avoid obstacles.
4.	Robotic Arm Control:
o	Use servo motors to control the robotic arm's movements and gripper for parcel handling.
Circuitry and Integration:
•	Motor Control: Use PWM signals to control motor speeds and direction.
•	Sensor Integration: Interface sensors (IMU, cameras, distance sensors) with the microcontroller.
•	Power Management: Ensure proper voltage regulation and battery monitoring.
Documentation:
1.	Project Overview:
o	Objectives, scope, and constraints of the project.
2.	Hardware Configuration:
o	Diagrams and specifications of the robot platform, arm, and sensors.
3.	Control Algorithms:
o	Detailed explanation of PID control for balance, path planning algorithms, and obstacle avoidance strategies.
4.	Code Implementation:
o	Include complete Python code snippets for PID control, path planning, obstacle avoidance, and robotic arm control.
5.	Testing and Validation:
o	Describe testing procedures, results, and validation steps using simulations or real-world trials.




import time
import math

# Constants for PID control
Kp = 10.0  # Proportional gain
Ki = 0.0   # Integral gain
Kd = 0.1   # Derivative gain

# Global variables for PID control
integral = 0.0
previous_error = 0.0

# Dummy function to simulate reading IMU data (replace with actual implementation)
def read_imu():
    # Simulating IMU data (angle in radians)
    return 0.0  # Replace with actual reading

# PID control function for maintaining balance
def pid_control(setpoint, current_value, dt):
    global integral, previous_error
    
    error = setpoint - current_value
    integral += error * dt
    derivative = (error - previous_error) / dt
    
    output = Kp * error + Ki * integral + Kd * derivative
    
    previous_error = error
    
    return output

# Function to simulate motor movement (replace with actual motor control)
def move_motors(speed):
    print(f"Moving motors with speed: {speed}")

# Main function to run the balancing algorithm
def main():
    # Initial setup
    setpoint = 0.0  # Desired angle for balancing (in radians)
    sample_time = 0.01  # Sample time for PID control (in seconds)
    
    while True:
        start_time = time.time()
        
        # Read IMU data (simulate reading angle)
        current_angle = read_imu()
        
        # Calculate PID output
        control_output = pid_control(setpoint, current_angle, sample_time)
        
        # Apply PID output to motors (simplified for illustration)
        move_motors(control_output)
        
        # Sleep to maintain loop frequency
        elapsed_time = time.time() - start_time
        time.sleep(max(0, sample_time - elapsed_time))

if __name__ == "__main__":
    main()
