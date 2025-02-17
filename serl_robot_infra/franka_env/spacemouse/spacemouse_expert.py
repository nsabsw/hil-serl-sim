import time
import multiprocessing
import numpy as np
import inputs
from franka_env.spacemouse import pyspacemouse
from typing import Tuple
from dataclasses import dataclass
from enum import Enum
from pynput import keyboard, mouse


class SpaceMouseExpert:
    """
    This class provides an interface to the SpaceMouse.
    It continuously reads the SpaceMouse state and provides
    a "get_action" method to get the latest action and button state.
    """

    def __init__(self):
        pyspacemouse.open()

        # Manager to handle shared state between processes
        self.manager = multiprocessing.Manager()
        self.latest_data = self.manager.dict()
        self.latest_data["action"] = [0.0] * 6  # Using lists for compatibility
        self.latest_data["buttons"] = [0, 0, 0, 0]

        # Start a process to continuously read the SpaceMouse state
        self.process = multiprocessing.Process(target=self._read_spacemouse)
        self.process.daemon = True
        self.process.start()

    def _read_spacemouse(self):
        while True:
            state = pyspacemouse.read_all()
            action = [0.0] * 6
            buttons = [0, 0, 0, 0]

            if len(state) == 2:
                action = [
                    -state[0].y, state[0].x, state[0].z,
                    -state[0].roll, -state[0].pitch, -state[0].yaw,
                    -state[1].y, state[1].x, state[1].z,
                    -state[1].roll, -state[1].pitch, -state[1].yaw
                ]
                buttons = state[0].buttons + state[1].buttons
            elif len(state) == 1:
                action = [
                    -state[0].y, state[0].x, state[0].z,
                    -state[0].roll, -state[0].pitch, -state[0].yaw
                ]
                buttons = state[0].buttons

            # Update the shared state
            self.latest_data["action"] = action
            self.latest_data["buttons"] = buttons

    def get_action(self) -> Tuple[np.ndarray, list]:
        """Returns the latest action and button state of the SpaceMouse."""
        action = self.latest_data["action"]
        buttons = self.latest_data["buttons"]
        return np.array(action), buttons
    
    def close(self):
        # pyspacemouse.close()
        self.process.terminate()

class ControllerType(Enum):
    PS5 = "ps5"
    XBOX = "xbox"

@dataclass
class ControllerConfig:
    resolution: dict
    scale: dict

class JoystickExpert:
    """
    This class provides an interface to the Joystick/Gamepad.
    It continuously reads the joystick state and provides
    a "get_action" method to get the latest action and button state.
    """

    CONTROLLER_CONFIGS = {
        ControllerType.PS5: ControllerConfig(
            # PS5 controller joystick values have 8 bit resolution [0, 255]
            resolution={
                'ABS_X': 2**8,
                'ABS_Y': 2**8,
                'ABS_RX': 2**8,
                'ABS_RY': 2**8,
                'ABS_Z': 2**8,
                'ABS_RZ': 2**8,
                'ABS_HAT0X': 1.0,
            },
            scale={
                'ABS_X': 0.4,
                'ABS_Y': 0.4,
                'ABS_RX': 0.5,
                'ABS_RY': 0.5,
                'ABS_Z': 0.8,
                'ABS_RZ': 1.2,
                'ABS_HAT0X': 0.5,
            }
        ),
        ControllerType.XBOX: ControllerConfig(
            # XBOX controller joystick values have 16 bit resolution [0, 65535]
            resolution={
                'ABS_X': 2**16,
                'ABS_Y': 2**16,
                'ABS_RX': 2**16,
                'ABS_RY': 2**16,
                'ABS_Z': 2**8,
                'ABS_RZ': 2**8,
                'ABS_HAT0X': 1.0,
            },
            scale={
                'ABS_X': -0.1,
                'ABS_Y': -0.1,
                'ABS_RX': 0.3,
                'ABS_RY': 0.3,
                'ABS_Z': 0.05,
                'ABS_RZ': 0.05,
                'ABS_HAT0X': 0.3,
            }
        ),
    }

    def __init__(self, controller_type=ControllerType.XBOX):
        self.controller_type = controller_type
        self.controller_config = self.CONTROLLER_CONFIGS[controller_type]

        # Manager to handle shared state between processes
        self.manager = multiprocessing.Manager()
        self.latest_data = self.manager.dict()
        self.latest_data["action"] = [0.0] * 6
        self.latest_data["buttons"] = [False, False]

        # Start a process to continuously read Joystick state
        self.process = multiprocessing.Process(target=self._read_joystick)
        self.process.daemon = True
        self.process.start()


    def _read_joystick(self):        
        action = [0.0] * 6
        buttons = [False, False]
        
        while True:
            try:
                # Get fresh events
                events = inputs.get_gamepad()
          
                # Process events
                for event in events:
                    if event.code in self.controller_config.resolution:
                        # Calculate relative changes based on the axis
                        # Normalize the joystick input values to range [-1, 1] expected by the environment
                        resolution = self.controller_config.resolution[event.code]
                        if self.controller_type == ControllerType.PS5:
                            normalized_value = (event.state - (resolution / 2)) / (resolution / 2)
                        else:
                            normalized_value = event.state / (resolution / 2)
                        scaled_value = normalized_value * self.controller_config.scale[event.code]

                        if event.code == 'ABS_Y':
                            action[0] = scaled_value
                        elif event.code == 'ABS_X':
                            action[1] = scaled_value
                        elif event.code == 'ABS_RZ':
                            action[2] = scaled_value
                        elif event.code == 'ABS_Z':
                            # Flip sign so this will go in the down direction
                            action[2] = -scaled_value
                        elif event.code == 'ABS_RX':
                            action[3] = scaled_value
                        elif event.code == 'ABS_RY':
                            action[4] = scaled_value
                        elif event.code == 'ABS_HAT0X':
                            action[5] = scaled_value
                        
                    # Handle button events
                    elif event.code == 'BTN_TL':
                        buttons[0] = bool(event.state)
                    elif event.code == 'BTN_TR':
                        buttons[1] = bool(event.state)

                # Update the shared state
                self.latest_data["action"] = action
                self.latest_data["buttons"] = buttons
                
            except inputs.UnpluggedError:
                print("No controller found. Retrying...")
                time.sleep(1)

    def get_action(self):
        """Returns the latest action and button state from the Joystick."""
        action = self.latest_data["action"]
        buttons = self.latest_data["buttons"]
        return np.array(action), buttons
    
    def close(self):
        self.process.terminate()

class KbdExpert:
    """
    This class provides an interface to the Kbd/pygame.
    It continuously reads the kbd state and provides
    a "get_action" method to get the latest action and button state.
    """

    def __init__(self):

        # Manager to handle shared state between processes
        self.manager = multiprocessing.Manager()
        self.latest_data = self.manager.dict()
        self.latest_data["action"] = [0.0] * 6
        self.latest_data["buttons"] = [False, False]
        self.control_speed = 0.05

        # Start a process to continuously read kbd state
        self.process = multiprocessing.Process(target=self. _read_keyboard_process)
        self.process.daemon = True
        self.process.start()
        print('keyboard control started')


    def _read_keyboard_process(self):
        with keyboard.Listener(on_press=self._key_press, on_release=self._key_release) as listener:
            listener.join()


    def _key_release(self, key):
        # Reset the action when the key is released
        self.latest_data["action"] = [0.0] * 6
        # You can also reset the buttons if needed
        self.latest_data["buttons"] = [False, False]


    def _key_press(self, key):
        action = [0.0] * 6
        buttons = [False, False]
        
        try:
            code = key.char # simple
        except:
            code = key.name # special

        # print(code)

        if code == 'up': # 前
            action[0] += self.control_speed
        elif code == 'down': # 后
            action[0] -= self.control_speed
        elif code == 'left': # 左
            action[1] += self.control_speed
        elif code == 'right': # 右
            action[1] -= self.control_speed
        elif code == '8': # 上
            action[2] += self.control_speed
        elif code == '2': # 下
            action[2] -= self.control_speed
        elif code == '7': # raw
            action[3] -= self.control_speed
        elif code == '9':
            action[3] += self.control_speed
        elif code == '4': # pitch
            action[4] -= self.control_speed
        elif code == '6':
            action[4] += self.control_speed
        elif code == '1': # yaw
            action[5] -= self.control_speed
        elif code == '3':
            action[5] += self.control_speed
        
        if code == 'ctrl':
            buttons[1] = True # close
        elif code == 'alt':
            buttons[0] = True # open

        # Update the shared state
        self.latest_data["action"] = action
        self.latest_data["buttons"] = buttons


    def get_action(self):
        """Returns the latest action and button state from the kbd."""
        action = self.latest_data["action"]
        buttons = self.latest_data["buttons"]
        return np.array(action), buttons
    
    def close(self):
        self.process.terminate()