from math import pi
from machine import Pin, PWM, Timer

class Motor:
    """
    Represents a motor connected to a motor controller that has a two-pin
    input. One pin drives the motor "forward", the other drives the motor
    "backward".

    :type forward: int
    :param forward:
        The GP pin that controls the "forward" motion of the motor. 
    
    :type backward: int
    :param backward:
        The GP pin that controls the "backward" motion of the motor. 
    
    :param bool pwm:
        If :data:`True` (the default), PWM pins are used to drive the motor. 
        When using PWM pins, values between 0 and 1 can be used to set the 
        speed.
    
    """
    def __init__(self, dir_pin, pwm_pin, enca_pin, encb_pin, ab_order=1, frequency=1000):
        # constants
        self.ab_order = ab_order  # 1: a trigger first; -1: b trigger first
        self.CPR = 48
        self.GEAR_RATIO = 46.85
        self.timer_period = 10  # millisecond
        self.K_P = 0.01
        self.K_I = 0.002
        self.K_D = 0

        # set pins
        self._dir_pin = Pin(dir_pin, Pin.OUT)
        self._pwm_pin = PWM(Pin(pwm_pin))
        self._enca_pin = Pin(enca_pin, Pin.IN, Pin.PULL_UP)
        self._encb_pin = Pin(encb_pin, Pin.IN, Pin.PULL_UP)
        self._pwm_pin.freq(frequency)
        self._enca_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._enca_handler)
        self._encb_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._encb_handler)
        # set timer to monitor motor velocity
        self._velocity_monitor = Timer(mode=Timer.PERIODIC, period=self.timer_period, callback=self._velmon_cb)
        # variables
        self.encoder_counts = 0
        self.prev_counts = 0
        self.velocity = 0.  # actual motor velocity
        self.target_vel = 0.
        self.enca_val = self._enca_pin.value()
        self.encb_val = self._encb_pin.value()
        self.err = 0.
        self.prev_err = 0.
        self.diff_err = 0.
        self.inte_err = 0.
        self.dutycycle = 0.

    def _enca_handler(self, pin):
        # self.encoder_counts, self.a_val
        self.enca_val = pin.value()
        if self.enca_val == 1:
            if self.encb_val == 0:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1
        else:
            if self.encb_val == 1:
                self.encoder_counts -= 1
            else:
                self.encoder_counts += 1

    def _encb_handler(self, pin):
        self.encb_val = pin.value()
        if self.encb_val == 1:
            if self.enca_val == 0:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1
        else:
            if self.enca_val == 1:
                self.encoder_counts += 1
            else:
                self.encoder_counts -= 1

    def _velmon_cb(self, timer):
        counts_diff = self.encoder_counts - self.prev_counts
        self.velocity = self.ab_order * counts_diff / (self.CPR * self.GEAR_RATIO * self.timer_period / 1000) * (2 * pi)
        self.prev_counts = self.encoder_counts

    def forward(self, duty=1.0):
        """
        Makes the motor turn "forward".

        :param float speed:
            The speed as a value between 0 and 1: 1 is full speed, 0 is stop. Defaults to 1.

        :param float t:
            The time in seconds that the motor should turn for. If None is 
            specified, the motor will stay on. The default is None.

        :param bool wait:
           If True, the method will block until the time `t` has expired. 
           If False, the method will return and the motor will turn on in
           the background. Defaults to False. Only effective if `t` is not
           None.
        """
        assert 0<=duty<=1
        self._dir_pin.value(0)
        self._pwm_pin.duty_u16(int(duty*65536))

    def backward(self, duty=1.0):
        """
        Makes the motor turn "forward".

        :param float speed:
            The speed as a value between 0 and 1: 1 is full speed, 0 is stop. Defaults to 1.

        :param float t:
            The time in seconds that the motor should turn for. If None is 
            specified, the motor will stay on. The default is None.

        :param bool wait:
           If True, the method will block until the time `t` has expired. 
           If False, the method will return and the motor will turn on in
           the background. Defaults to False. Only effective if `t` is not
           None.
        """
        assert 0<=duty<=1
        self._dir_pin.value(1)
        self._pwm_pin.duty_u16(int(duty*65536))

    def stop(self):
        """
        Makes the motor stop.
        """
        self._pwm_pin.duty_u16(0)
        self.err = 0.
        self.prev_err = 0.
        self.inte_err = 0.

    def set_velocity(self, target_vel):
        self.target_vel = target_vel
        self.err = target_vel - self.velocity
        self.diff_err = self.err - self.prev_err
        self.inte_err += self.diff_err
        self.prev_err = self.err
        self.dutycycle += self.K_P * self.err + self.K_D * self.diff_err + self.K_I * self.inte_err
        if self.dutycycle > 0:
            if self.dutycycle > 1:
                self.dutycycle = 1
            self.forward(duty=self.dutycycle)
        elif self.dutycycle < 0:
            if self.dutycycle < -1:
                self.dutycycle = -1
            self.backward(duty=-self.dutycycle)
        else:
            self.stop()


class Robot:
    
    def __init__(self, left_motor_pins, right_motor_pins, frequency=1000):
        assert len(left_motor_pins)==4
        assert len(right_motor_pins)==4
        self._left_motor = Motor(
            dir_pin=left_motor_pins[0], 
            pwm_pin=left_motor_pins[1], 
            enca_pin=left_motor_pins[2], 
            encb_pin=left_motor_pins[3], 
            ab_order=1, 
            frequency=frequency
        )
        self._right_motor = Motor(
            dir_pin=right_motor_pins[0], 
            pwm_pin=right_motor_pins[1], 
            enca_pin=right_motor_pins[2], 
            encb_pin=right_motor_pins[3], 
            ab_order=-1, 
            frequency=frequency
        )
        # constants
        self.WHEEL_RADIUS = 0.0375
        self.WHEEL_SEPARATION = 0.19
        self.timer_period = 10  # ms
        # variables
        self.linear_velocity = 0.  # actual linear velocity
        self.angular_velocity = 0.  # actual angular velocity
        self.target_lin = 0.  # target velocity
        self.target_ang = 0.
        # set a timer to monitor robot velocity
        self._velocity_monitor = Timer(mode=Timer.PERIODIC, period=self.timer_period, callback=self._velmon_cb)

    def _velmon_cb(self, timer):
        self.left_lin_vel = self._left_motor.velocity * self.WHEEL_RADIUS
        self.right_lin_vel = self._right_motor.velocity * self.WHEEL_RADIUS
        self.linear_velocity = (self.left_lin_vel + self.right_lin_vel) / 2
        self.angular_velocity = (self.right_lin_vel - self.left_lin_vel) / self.WHEEL_SEPARATION

    def forward(self, speed=1.0):
        self._left_motor.forward(duty=speed)
        self._right_motor.forward(duty=speed)

    def backward(self, speed=1.0):
        self._left_motor.backward(duty=speed)
        self._right_motor.backward(duty=speed)

    def stop(self):
        self._left_motor.stop()
        self._right_motor.stop()

    def set_velocity(self, target_lin, target_ang):
        self.target_lin = target_lin
        self.target_ang = target_ang
        left_target_vel = (target_lin - (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS
        right_target_vel = (target_lin + (target_ang * self.WHEEL_SEPARATION) / 2) / self.WHEEL_RADIUS
        self._left_motor.set_velocity(left_target_vel) 
        self._right_motor.set_velocity(right_target_vel)
