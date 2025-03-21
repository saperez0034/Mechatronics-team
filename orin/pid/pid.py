
# Code adapted from: https://www.digikey.com/en/maker/tutorials/2024/implementing-a-pid-controller-algorithm-in-python
def pid_controller(target, current, kp, ki, kd, previous_error, integral, dt):
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

