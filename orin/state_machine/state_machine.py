import vision.detect_lesion as detect_lesion
import stm32.stm32_serial as stm32
import time
import pid.pid

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
        self.needle_x = 320
        self.needle_y = 240
        self.lesion_midpoint = None
        self.pid_error_x = 0
        self.pid_error_y = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.i_x = 0
        self.i_y = 0
        self.dt = 0.1
        self.stepx = 0
        self.stepy = 0
        self.lesions = None
        self.lesion_index = 0
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

    def move_x (self, steps):
        stm32.send_data(self.ser, 'stepx '+ str(steps) + '\r')

    def move_y (self, steps):
        stm32.send_data(self.ser, 'stepy '+ str(steps) + '\r')

    def detecting_state(self):
        print("Detecting state")
        color_image = detect_lesion.get_color_image(self.pipeline)
        result = detect_lesion.detect_and_log_grape_properties(color_image)
        # Transition to the next state
        if result is None:
            stm32.send_data(self.ser, self.stepperY_command)
            self.current_state = 'DETECTING'
        else:
            self.lesions = result
            # self.h_range, self.s_range, self.v_range, self.lesion_midpoint = result
            # # print("Detected Grape Color Ranges (HSV):")
            # # print(f"Hue: {self.h_range[0]:.2f} - {self.h_range[1]:.2f}")
            # # print(f"Saturation: {self.s_range[0]:.2f} - {self.s_range[1]:.2f}")
            # # print(f"Value: {self.v_range[0]:.2f} - {self.v_range[1]:.2f}")
            # pid.pid_controller(self.needle_x, self.lesion_midpoint[0], self.kp, self.ki, self.kd, self.pid_error_x, )
            self.current_state = 'PROCESSING'

    def processing_state(self):
        print("Processing state")
        self.lesion_midpoint = self.lesions[self.lesion_index][3]
        self.step_x, self.pid_error_x, self.i_x = pid.pid_controller(self.needle_x, self.lesion_midpoint[0], self.kp, self.ki, self.kd, self.pid_error_x, self.i_x, self.dt)
        self.step_y, self.pid_error_x, self.i_x = pid.pid_controller(self.needle_y, self.lesion_midpoint[0], self.kp, self.ki, self.kd, self.pid_error_y, self.i_y, self.dt)
        self.move_x(self, self.step_x)
        self.move_y(self, self.step_y)
        self.lesion_index = self.lesion_index + 1
        if (self.lesion_index == self.lesions.len()):
            self.current_state = 'FINAL'

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
    