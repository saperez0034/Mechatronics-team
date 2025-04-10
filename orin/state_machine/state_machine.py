import vision.detect_lesion as detect_lesion
import stm32.stm32_serial as stm32
import time
from pid.pid_controller import pid_controller

class StateMachine:
    def __init__(self):
        self.states = {
            'INITIAL': self.initial_state,
            'DETECTING': self.detecting_state,
            'PROCESSING': self.processing_state,
            'BREATHING': self.breathing_state,
            'EXTRACT_SAMPLE': self.extract_sample_state,
            'FINAL': self.final_state
        }
        self.current_state = 'INITIAL'
        self.pipeline = None
        self.ser = None
        self.needle_x = 250
        self.needle_y = 336
        self.pid_error_x = 0
        self.pid_error_y = 0
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.i_x = 0
        self.i_y = 0
        self.dt = 0.1
        self.step_x = 0
        self.step_y = 0
        self.total_steps_x = 0
        self.total_steps_y = 0
        self.stepperX_midpoint_steps = 100
        self.stable_location = None
        self.current_location = None

    def move_x (self, steps):
        stm32.send_data(self.ser, 'stepx '+ str(steps) + '\r')

    def move_y (self, steps):
        stm32.send_data(self.ser, 'stepy '+ str(steps) + '\r')

    def lin_servo(self, pctg):
        stm32.send_data(self.ser, 'lin_servo '+ str(pctg) + '\r')

    def lin_act(self, ext):
        stm32.send_data(self.ser, 'lin_act '+ str(ext) + '\r')

    def initial_state(self):
        print("Initial state")
        self.pipeline = detect_lesion.vision_setup()
        self.ser = stm32.stm32_setup()
        for i in range(3):
            stm32.send_data(self.ser, "\r")
        
        # self.move_x(-1000000) # Resetting the End effector to Origin
        # self.move_y(-1000000)

        self.lin_servo(0) # Moving end effector to the top
        self.lin_act(0) # Priming needle

        time.sleep(2)

        self.move_x(self.stepperX_midpoint_steps)
        self.total_steps_x += self.stepperX_midpoint_steps
        
        self.current_state = 'DETECTING'

    def detecting_state(self):
        print("Detecting state")
        result = detect_lesion.detect_stable_location(self.pipeline)
        # Transition to the next state
        if result == []:
            self.move_y(10)
            self.total_steps_y += 10
            self.current_state = 'DETECTING'
        else:
            # self.lesion_midpoint = result[0][3]
            self.stable_location = result
            self.current_state = 'PROCESSING'

    def processing_state(self):
        print("Processing state")
        if (self.pid_error_x > 0.1 or self.pid_error_x == 0):
           self.step_x, self.pid_error_x, self.i_x = pid_controller(
                       self.needle_x, self.stable_location[0], self.kp, self.ki, 
                       self.kd, self.pid_error_x, self.i_x, self.dt)
           self.move_x(self, self.step_x)
           self.total_steps_x += self.step_x
        if (self.pid_error_y > 0.1 or self.pid_error_y == 0):
            self.step_y, self.pid_error_y, self.i_y = pid_controller(
                        self.needle_y, self.stable_location[1], self.kp, self.ki, 
                        self.kd, self.pid_error_y, self.i_y, self.dt)
            print(self.pid_error_y)
            self.move_y(self.step_y)
            self.total_steps_y += self.step_y
        if (self.pid_error_x < 10 and self.pid_error_y < 10):
            self.lin_servo(80)
            self.current_state = 'BREATHING'
        else: 
            self.current_state = 'DETECTING'

    def breathing_state(self):
        result = detect_lesion.detect(self.pipeline)
        if detect_lesion.same_location(result, self.stable_location):
            self.current_state = 'EXTRACT_SAMPLE'
        else:
            self.current_state = 'BREATHING'

    def extract_sample_state(self):
        print("Extracting sample state")
        self.lin_servo(100)
        time.sleep(.5)
        self.lin_act(1)
        time.sleep(0.5)
        self.lin_stepper(0)
        time.sleep(10)
        self.current_state = 'FINAL'

    def final_state(self):
        print("Final state")
        self.move_x(-self.total_steps_x)
        self.move_y(-self.total_steps_y)
        time.sleep(30)
        self.lin_act(self, 0)
    
    def run(self):
        while True:
            state_function = self.states[self.current_state]
            state_function()
            time.sleep(0.1)

if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.run()
    
