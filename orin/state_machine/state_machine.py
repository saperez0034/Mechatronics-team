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
        self.needle_x = 334 #y axis irl
        self.needle_y = 323 #x axis irl
        self.pid_error_x = 0
        self.pid_error_y = 0
        self.kp = 1
        self.ki = 0.5
        self.kd = 0.2
        self.i_x = 0
        self.i_y = 0
        self.dt = 0.1
        self.step_x = 0
        self.step_y = 0
        self.y_dir = True
        self.x_dir = True
        self.xy_limit = 4500
        self.total_steps_x = 0
        self.total_steps_y = 0
        self.stepperX_midpoint_steps = 100
        self.stable_location = None
        self.current_location = None

    def move_x(self, steps):
        stm32.send_data(self.ser, 'stepx ' + str(steps) + '\r')

    def move_y(self, steps):
        stm32.send_data(self.ser, 'stepy ' + str(steps) + '\r')

    def lin_servo(self, pctg):
        stm32.send_data(self.ser, 'servo_lin ' + str(pctg) + '\r')

    def lin_act(self, ext):
        stm32.send_data(self.ser, 'lin_act ' + str(ext) + '\r')

    def initial_state(self):
        print("Initial state")
        self.pipeline = detect_lesion.vision_setup()
        self.ser = stm32.stm32_setup()
        for i in range(3):
            stm32.send_data(self.ser, "\r")

        self.move_x(-5000) # Resetting the End effector to Origin
        self.move_y(-5000)
        time.sleep(5)
        self.lin_servo(90)
        time.sleep(1)
        self.lin_servo(0)  # Moving end effector to the top
        self.lin_act(1)
        self.move_y(2700)
        self.total_steps_y = 2700
        self.move_x(1000)
        self.total_steps_x = 1000
        time.sleep(5)

        self.current_state = 'DETECTING'
        print("Detecting state")

    def detecting_state(self):
        time.sleep(0.2)
        result = detect_lesion.detect_stable_location(self.pipeline)
        # Transition to the next state
        if result == (-1, -1):
            print("nothing")
            self.move_x(100 if self.x_dir else -100)
            self.total_steps_x += (100 if self.x_dir else -100)
            if self.total_steps_x >= self.xy_limit or self.total_steps_x <= 0:
                self.x_dir = not self.x_dir
                self.move_y(900 if self.y_dir else -900)
                self.total_steps_y += (900 if self.y_dir else -900)
                self.y_dir = not self.y_dir if self.total_steps_y >= self.xy_limit or self.y_dir <= 0 else self.y_dir
                time.sleep(2)
            self.current_state = 'DETECTING'
        else:
            # self.lesion_midpoint = result[0][3]
            self.stable_location = result
            print("stable:", result)
            self.current_state = 'PROCESSING'

    def processing_state(self):
        ERROR_LIM = 7
        print("Processing state")
        self.step_x, self.pid_error_x, self.i_x = pid_controller(
            self.needle_x, self.stable_location[0], self.kp, self.ki,
            self.kd, self.pid_error_x, self.i_x, self.dt)
        print(f"y error:{self.pid_error_x}")
        if (abs(self.pid_error_x) >= ERROR_LIM or self.pid_error_x == 0):
            if (self.step_x + self.total_steps_y >= self.xy_limit):
                self.step_x = self.xy_limit - self.total_steps_y
                self.y_dir = not self.x_dir
                print("error in y high lim")
            if (self.step_x + self.total_steps_y <= 0):
                self.step_x = -1 * self.total_steps_y
                self.y_dir = not self.x_dir
                print("error in y low lim")
            print("move y: "+ str(self.step_x))
            self.move_y(self.step_x)
            self.total_steps_y += self.step_x
    
        self.step_y, self.pid_error_y, self.i_y = pid_controller(
            self.needle_y, self.stable_location[1], self.kp, self.ki,
            self.kd, self.pid_error_y, self.i_y, self.dt)
        print(f"x error:{self.pid_error_y}")
        if (abs(self.pid_error_y) >= ERROR_LIM or self.pid_error_y == 0):
            if (self.step_y + self.total_steps_x >= self.xy_limit):
                self.step_y = self.xy_limit - self.total_steps_x
                self.x_dir = not self.y_dir
                print("error in x high lim")
            if (self.step_y + self.total_steps_x <= 0):
                self.step_y = -1 * self.total_steps_x
                self.x_dir = not self.y_dir
                print("error in x low lim")
            print("move x: "+ str(self.step_y))
            self.move_x(self.step_y)
            self.total_steps_x += self.step_y
        time.sleep(1.5)

        if(abs(self.pid_error_x) < ERROR_LIM and abs(self.pid_error_y) < ERROR_LIM):
            print(self.stable_location)
            self.current_state = 'BREATHING'
            print("Breathing")
        else:
            counter = 0
            self.stable_location = detect_lesion.detect_stable_location(self.pipeline)
            while (self.stable_location == (-1, -1)):
                self.stable_location = detect_lesion.detect_stable_location(self.pipeline)
                counter += 1
                if counter == 1000:
                    self.current_state = 'DETECTING'
                    self.pid_error_x = 0
                    self.pid_error_y = 0
                    print("DETECTING")
                    break
                pass

    def breathing_state(self):
        self.lin_servo(25)
        self.lin_act(0)
        time.sleep(2)
        img = detect_lesion.get_color_image(self.pipeline)
        result = detect_lesion.detect(img)
        if detect_lesion.same_location(result, self.stable_location) != (-1, -1):
            self.current_state = 'EXTRACT_SAMPLE'
        else:
            self.current_state = 'BREATHING'

    def extract_sample_state(self):
        print("Extracting sample state")
        self.lin_servo(38)
        time.sleep(1.5)
        self.lin_act(1)
        time.sleep(1)
        self.lin_servo(0)
        time.sleep(2)
        self.move_x(-self.total_steps_x)
        self.move_y(-self.total_steps_y)
        self.current_state = 'FINAL'

    def final_state(self):
        print("Final state")
        self.total_steps_x == 0
        self.total_steps_y == 0
        time.sleep(5)
        self.lin_act(0)

    def run(self):
        while True:
            state_function = self.states[self.current_state]
            state_function()
            time.sleep(0.1)


if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.run()
