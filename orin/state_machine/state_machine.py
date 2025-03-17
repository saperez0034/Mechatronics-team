import vision.detect_lesion as detect_lesion
import stm32.stm32_serial as stm32
import time

class StateMachine:
    def __init__(self):
        self.states = {
            'INITIAL': self.initial_state,
            'DETECTING': self.detecting_state,
            'PROCESSING': self.processing_state,
            'EXTRACT_SAMPLE': self.extract_sample_state,
            'FINAL': self.final_state
        }
        self.current_state = 'INITIAL'
        self.pipeline = None
        self.ser = None
        self.h_range = None
        self.s_range = None
        self.v_range = None
        self.led_command = "toggleled\r"
        self.lin_act_extend_command = "lin_act 1\r"
        self.lin_act_retract_command = "lin_act 0\r"
        self.stepperY_command = "stepY 10\r"
        self.stepperX_command = "stepX 10\r"
        self.stepperX_midpoint_steps = 100

    def initial_state(self):
        print("Initial state")
        # Transition to the next state
        self.pipeline = detect_lesion.vision_setup()
        self.ser = stm32.stm32_setup()
        for i in range(3):
            stm32.send_data(self.ser, "\r")
        stm32.send_data(self.ser, self.lin_act_extend_command)
        self.current_state = 'DETECTING'

    def detecting_state(self):
        print("Detecting state")
        color_image = detect_lesion.get_color_image(self.pipeline)
        result = detect_lesion.detect_and_log_grape_properties(color_image)
        # Transition to the next state
        if result is None:
            stm32.send_data(self.ser, self.stepperY_command)
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
        stm32.send_data(self.ser, self.led_command)
        self.current_state = 'FINAL'
        time.sleep(5)

    def extract_sample_state(self):
        print("Extracting sample state")
        # Add extraction logic here
        # Transition to the next state
        stm32.send_data(self.ser, self.lin_act_retract_command)

    def release_sample(self):
        print("Releasing sample state")
        # Add release logic here
        # Transition to the next state
        stm32.send_data(self.ser, self.lin_act_extend_command)

    def final_state(self):
        print("Final state")
        # Add finalization logic here
        # Transition to the next state or end
        stm32.send_data(self.ser, self.led_command)
        self.current_state = 'DETECTING'

    def run(self):
        while True:
            state_function = self.states[self.current_state]
            state_function()
            time.sleep(0.1)


if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.run()
    