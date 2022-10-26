from inputs import get_gamepad
import math
import threading
import serial as sr
import time

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self):
        RJoyX = self.RightJoystickX
        RJoyY = self.RightJoystickY
        if RJoyX <= 0.05 and RJoyX >= -0.05 :
            RJoyX = 0
        elif RJoyX >= 0.995 :
            RJoyX = 1
        elif RJoyX <= -0.995 :
            RJoyX = -1
        if RJoyY <= 0.05 and RJoyY >= -0.05 :
            RJoyY = 0
        elif RJoyY >= 0.995 :
            RJoyY = 1
        elif RJoyY <= -0.995 :
            RJoyY = -1
        return [RJoyX, RJoyY]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL 
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL 
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_WEST':
                    self.X = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

ser2 = sr.Serial('COM7', 115200, timeout=5)
Max_PMW = 50

joy = XboxController()
while True:
    j = joy.read()
    Rpwm = int(j[1]*Max_PMW) - int(j[0]*Max_PMW)
    Lpwm = int(j[1]*Max_PMW) + int(j[0]*Max_PMW)
    if Rpwm > Max_PMW :
        Rpwm = Max_PMW
    elif Rpwm < -Max_PMW :
        Rpwm = -Max_PMW
    if Lpwm > Max_PMW :
        Lpwm = Max_PMW
    elif Lpwm < -Max_PMW :
        Lpwm = -Max_PMW
    if Lpwm >= 0 :
        if Lpwm >= 100 :
            charL = "+" + str(Lpwm)
        if Lpwm < 100 and Lpwm >=10 :
            charL = "+0" + str(Lpwm)
        if Lpwm < 10 :
            charL = "+00" + str(Lpwm)
    if Lpwm < 0 :
        Lpwm = -Lpwm
        if Lpwm >= 100 :
            charL = "-" + str(Lpwm)
        if Lpwm < 100 and Lpwm >=10 :
            charL = "-0" + str(Lpwm)
        if Lpwm < 10 :
            charL = "-00" + str(Lpwm)
    if Rpwm >= 0 :
        if Rpwm >= 100 :
            charR = "+" + str(Rpwm)
        if Rpwm < 100 and Rpwm >=10 :
            charR = "+0" + str(Rpwm)
        if Rpwm < 10 :
            charR = "+00" + str(Rpwm)
    if Rpwm < 0 :
        Rpwm = -Rpwm
        if Rpwm >= 100 :
            charR = "-" + str(Rpwm)
        if Rpwm < 100 and Rpwm >=10 :
            charR = "-0" + str(Rpwm)
        if Rpwm < 10 :
            charR = "-00" + str(Rpwm)
    char = charL + charR +'\n'
    ser2.write(str.encode(char))
    time.sleep(0.005)
    print(char)
