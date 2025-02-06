import serial
import PySimpleGUI as sg

# Function to send motor control commands to Arduino
def send_motor_commands(servo_pos, stepper_angle, dc_motor_pos, dc_motor_vel):
    # Construct the message in the expected format
    message = f"{servo_pos},{stepper_angle},{dc_motor_pos},{dc_motor_vel}\n"
    arduino.write(message.encode())

if __name__ == "__main__":
    # Setup serial communication (adjust COM port to your Arduino's port)
    arduino = serial.Serial('/dev/cu.usbmodem2101', 9600, timeout=1)  # Replace 'COM3' with your Arduino's serial port
    # Create the layout for the GUI
    layout = [
        [sg.Text("Servo Motor (Position Control)", font=("Helvetica", 14))],
        [sg.Text("Servo Position (0 to 180°):", size=(20, 1)), 
        sg.Slider(range=(0, 180), orientation='h', size=(40, 20), default_value=90, key='servo_position')],
        
        [sg.Text("Stepper Motor (Angle Control)", font=("Helvetica", 14))],
        [sg.Text("Motor Speed (-360° to 360°):", size=(20, 1)), 
        sg.Slider(range=(-360, 360), orientation='h', size=(40, 20), default_value=0, key='stepper_angle')],
        
        [sg.Text("DC Motor (Position and/or Velocity Control)", font=("Helvetica", 14))],
        [sg.Text("Select Control Mode for DC Motor:", size=(20, 1))],
        [sg.Radio("Velocity Control", "DC_MOTOR_MODE", default=True, key="dc_control"),
        sg.Radio("Position Control", "DC_MOTOR_MODE", key="dc_position_control")],
        
        [sg.Text("Motor Speed (-100 to 100):", size=(20, 1)), 
        sg.Slider(range=(-100, 100), orientation='h', size=(40, 20), default_value=0, key="dc_motor_velocity")],
        
        [sg.Text("Motor Position (-360° to 360°):", size=(20, 1)), 
        sg.Slider(range=(-360, 360), orientation='h', size=(40, 20), default_value=0, key="dc_motor_position")],
        
        [sg.Button("Update Motor Positions", size=(20, 1))],
        [sg.Exit(size=(20, 1))]
    ]
    # Create the window
    window = sg.Window("Motor Control Interface", layout, finalize=True)
    # Main loop
    while True:
        event, values = window.read()
        if event == sg.WINDOW_CLOSED or event == 'Exit':
            break
        # Control logic for DC motor - Switching between Velocity and Position modes
        if values["dc_control"]:
            # Enable velocity control, disable position control
            window['dc_motor_velocity'].update(disabled=False)
            window['dc_motor_position'].update(disabled=True)
            dc_motor_vel = values['dc_motor_velocity']
            dc_motor_pos = 0  # Not using position control when in velocity mode
        elif values["dc_position_control"]:
            # Enable position control, disable velocity control
            window['dc_motor_position'].update(disabled=False)
            window['dc_motor_velocity'].update(disabled=True)
            dc_motor_pos = values['dc_motor_position']
            dc_motor_vel = 0  # Not using velocity control when in position mode
        # Handle the update button click
        if event == 'Update Motor Positions':
            servo_pos = values['servo_position']
            stepper_angle = values['stepper_angle']
            # Send motor control data to Arduino
            send_motor_commands(servo_pos, stepper_angle, dc_motor_pos, dc_motor_vel)
    # Close the window
    window.close()
    arduino.close()
