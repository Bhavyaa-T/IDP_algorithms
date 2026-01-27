from machine import Pin, PWM, ADC
import utime

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        
    def Stop(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, speed=100):
        self.mDir.value(0)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

    def Reverse(self, speed=30):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

class PID:
    def __init__ (self, Kp, Ki, Kd, output_limits = (-100, 100)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.min_out, self.max_out = output_limits
        self.I = 0.0
        self.e_prev = 0
        self.t_prev = utime.ticks_ms()
    
    def update(self, e):
        t = utime.ticks_ms()
        dt = utime.ticks_diff(t, self.t_prev) / 1000    # dividing by 1000 to convert to seconds
        self.t_prev = t
        if dt <= 0:
            dt = 0.001  # protection from divide by 0 error
        
        self.I = e * dt
        D = (e - self.e_prev) / dt
        self.e_prev = e

        update = self.Kp * e + self.Ki * self.I + self.Kd * D

        if update < self.min_out: 
            return self.min_out
        if update > self.max_out:
            return self.max_out
        return update
    


WEIGHTS = [-2.0, -1.0, 1.0, 2.0] #  These will be used for finding our centroid (negative for left, positive for right)

# ASSUMING BLACK IS HIGH 
digital_pins = [10, 11, 12, 13]  #GPIO pin numbers
digital = [Pin(i, Pin.IN) for i in digital_pins]

def read_sensors(digital):
    vals = []
    for pin in digital:
        vals.append(float(pin.value()))
    return vals

def centroid_position(vals, weights):

    val_weight_sum = 0.0
    val_sum = 0.0

    for vi, wi in zip(vals, weights):
        val_weight_sum += vi * wi
        val_sum += vi
    
    return val_weight_sum / val_sum, val_sum

        
def main():

    left_motor = Motor(dirPin = 4, PWMPin = 5)    
    right_motor = Motor(dirPin = 7, PWMPin = 6)

    pid = PID(Kp=0.5, Ki=0.5, Kd=0.5, output_limits=(-30,30))

    base_speed = 50     # This is a % of the max speed

    while True:

        sensor_vals = read_sensors(digital)

        pos, sum = centroid_position(sensor_vals, WEIGHTS)

        e = -pos
        correction = pid.update(e)

        cmd_left = base_speed + correction
        cmd_right = base_speed - correction

        cmd_left = max(0, min(100, cmd_left))
        cmd_right = max(0, min(100, cmd_right))

        left_motor.Forward(cmd_left)
        right_motor.Forward(cmd_right)

        utime.sleep(0.01)






















    




