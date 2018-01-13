from gpiozero import Button

class RotaryEncoder:
    """
    Decode mechanical rotary encoder pulses.

    The following example will print a Rotary Encoder change direction

        from gpiozero import RotaryEncoder

        def change(value):
            if value == 1:
                print("clockwise")
            else  # if change == -1
                print("counterclockwise")

        rotary = RotaryEncoder(2, 3)
        rotary.when_rotated = change

    Based in http://abyz.co.uk/rpi/pigpio/examples.html#Python_rotary_encoder_py
    """
    gpioA = None
    gpioB = None

    levA = 0
    levB = 0

    lastGpio = None

    when_rotated = lambda *args : None

    def __init__(self, pinA, pinB, pull_up=False):
        """
        Uses for dettect rotary encoder changes (set when_rotated attribute)
        It takes one parameter which is +1 for clockwise and -1 for counterclockwise.

        :param pinA int : 
        :param pinB int : 
        :pull_up bool :
            The common contact should be NOT connected to ground?
        """
        self.gpioA = Button(pinA, pull_up)
        self.gpioB = Button(pinB, pull_up)

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.gpioA.when_pressed = lambda *args : self.pulse(self.gpioA, 1)
        self.gpioA.when_released = lambda *args : self.pulse(self.gpioA, 0)

        self.gpioB.when_pressed = lambda *args : self.pulse(self.gpioB, 1)
        self.gpioB.when_released = lambda *args :  self.pulse(self.gpioB, 0)

    def pulse(self, gpio, level):
        """
        Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
        """
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:
            self.lastGpio = gpio
            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.when_rotated(1)
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.when_rotated(-1)
            else:
                if self.levB == 1:
                    self.when_rotated(-1)
                elif self.levA == 1:
                    self.when_rotated(1)

        else:
            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.when_rotated(1)
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.when_rotated(-1)
            else:
                if self.levB == 1:
                    self.when_rotated(-1)
                elif self.levA == 1:
                    self.when_rotated(1)