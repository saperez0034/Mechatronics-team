import vision.detect_lesion as detect_lesion
import motor.motor_control as motor_control
import time

class StateMachine:
    def __init__(self):
        self.states = {
            'INITIAL': self.initial_state,
            'DETECTING': self.detecting_state,
            'PROCESSING': self.processing_state,
            'FINAL': self.final_state
        }
        self.current_state = 'INITIAL'
        self.ser = None
        self.h_range = None
        self.s_range = None
        self.v_range = None
        self.led_command = "toggleled\r"

    def initial_state(self):
        print("Initial state")
        # Transition to the next state
        detect_lesion.vision_setup()
        self.ser = motor_control.motor_setup()
        self.current_state = 'DETECTING'

    def detecting_state(self):
        print("Detecting state")
        result = detect_lesion.detect_and_log_grape_properties()
        # Transition to the next state
        if result is None:
            self.current_state = 'DETECTING'
        else:
            self.h_range, self.s_range, self.v_range = result
            print("Detected Grape Color Ranges (HSV):")
            print(f"Hue: {self.h_range[0]:.2f} - {self.h_range[1]:.2f}")
            print(f"Saturation: {self.s_range[0]:.2f} - {self.s_range[1]:.2f}")
            print(f"Value: {self.v_range[0]:.2f} - {self.v_range[1]:.2f}")
            self.current_state = 'PROCESSING'

    def processing_state(self):
        print("Processing state")
        # Add processing logic here
        # Transition to the next state
        motor_control.send_data(self.ser, self.led_command)
        self.current_state = 'FINAL'
        time.sleep(5)

    def final_state(self):
        print("Final state")
        # Add finalization logic here
        # Transition to the next state or end
        motor_control.send_data(self.ser, self.led_command)
        self.current_state = 'INITIAL'

    def run(self):
        while True:
            state_function = self.states[self.current_state]
            state_function()
            time.sleep(0.1)


if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.run()