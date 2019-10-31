import rospy
from smach import State
from sensor_msgs.msg import Joy
import numpy as np
from src.competition3.scripts.util import notify_number


class JoystickInput:
    def __init__(self):
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.last_time = None
        self.debounce_interval = 0.1
        self.button_names = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
        }
        self.last_pressed = None

    def joy_callback(self, message):  # type: (Joy) -> None
        if self.last_time is None:
            self.last_time = rospy.get_time()

        if rospy.get_time() - self.last_time <= self.debounce_interval:
            return

        button_code = np.argmax(message.buttons)
        if message.buttons[button_code] == 1:
            if button_code in self.button_names:
                self.last_pressed = self.button_names[button_code]

    def wait_for_press(self):  # type: () -> str
        self.last_pressed = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.last_pressed is not None:
                return str(self.last_pressed)
            rate.sleep()


class SelectNumberState(State):
    def __init__(self, min, max):
        super(SelectNumberState, self).__init__(outcomes=['ok'], output_keys=['number'])
        self.number = min
        self.min = min
        self.max = max
        self.joystick = JoystickInput()

    def execute(self, ud):
        self.number = self.min
        while not rospy.is_shutdown():
            print('NUMBER IS {}'.format(self.number))
            notify_number(self.number)
            button = self.joystick.wait_for_press()
            if button == 'RB':
                self.number += 1
            elif button == 'LB':
                self.number -= 1
            elif button == 'A':
                ud.number = self.number
                return 'ok'
            if self.number < self.min:
                self.number = self.max
            elif self.number > self.max:
                self.number = self.min


if __name__ == '__main__':
    rospy.init_node('select_number')
    SelectNumber().execute({})
