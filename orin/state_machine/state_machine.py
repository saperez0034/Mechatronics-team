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
        self.led_command = "toggleled\r"
        self.lin_act_extend_command = "lin_act 1\r"
        self.lin_act_retract_command = "lin_act 0\r"
        self.lin_stepper_extend_command = "lin_stepper 100\r"
        self.lin_stepper_retract_command = "lin_stepper 0\r"
        self.stepperY_command = "stepY 10\r"
        self.stepperX_command = "stepX 10\r"
        self.stepperX_midpoint_steps = 100

    def move_x (self, steps):
        stm32.send_data(self.ser, 'stepx '+ str(steps) + '\r')

    def move_y (self, steps):
        stm32.send_data(self.ser, 'stepy '+ str(steps) + '\r')

    def release_sample(self):
        stm32.send_data(self.ser, self.lin_act_extend_command)

    def initial_state(self):
        print("Initial state")
        # Transition to the next state
        self.pipeline = detect_lesion.vision_setup()
        self.ser = stm32.stm32_setup()
        for i in range(3):
            stm32.send_data(self.ser, "\r")
        stm32.send_data(self.ser, self.lin_stepper_retract_command)
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
            self.lesion_midpoint = result
            self.current_state = 'PROCESSING'

    def processing_state(self):
        print("Processing state")
        if (self.pid_error_x > 0.1):
            self.step_x, self.pid_error_x, self.i_x = pid.pid_controller(
                        self.needle_x, self.lesion_midpoint[0], self.kp, self.ki, 
                        self.kd, self.pid_error_x, self.i_x, self.dt)
            self.move_x(self, self.step_x)
        if (self.pid_error_y > 0.1):
            self.step_y, self.pid_error_x, self.i_x = pid.pid_controller(
                        self.needle_y, self.lesion_midpoint[0], self.kp, self.ki, 
                        self.kd, self.pid_error_y, self.i_y, self.dt)
            self.move_y(self, self.step_y)
        if (self.pid_error_x < 0.1 and self.pid_error_y < 0.1):
            self.current_state = 'EXTRACT_SAMPLE'
        self.current_state = 'DETECTING'

    def extract_sample_state(self):
        print("Extracting sample state")
        # Add extraction logic here
        # Transition to the next state
        stm32.send_data(self.ser, self.lin_stepper_extend_command)
        time.sleep(10)
        stm32.send_data(self.ser, self.lin_act_retract_command)
        time.sleep(1)
        stm32.send_data(self.ser, self.lin_stepper_retract_command)
        time.sleep(10)
        self.current_state = 'FINAL'

    def final_state(self):
        print("Final state")
        # Add finalization logic here
        # Transition to the next state or end
        self.release_sample()
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
    