from pynput import keyboard, mouse
import debugpy
try:
    # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
    debugpy.listen(("localhost", 9501))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()
except Exception as e:
    pass

# # 键盘监听
# def on_keyboard_press(key):
#     print(key)
#     # try:
#     #     print(f'Key {key.char} was pressed.')
#     # except AttributeError:
#     #     print(f'Special key {key} was pressed.')

# keyboard_listener = keyboard.Listener(on_press=on_keyboard_press)
# keyboard_listener.start()

# 鼠标监听
def on_mouse_click(x, y, button, pressed):
    if pressed:
        print(f"Mouse {button} was clicked at ({x}, {y}).")

mouse_listener = mouse.Listener(on_click=on_mouse_click)
mouse_listener.start()

# 保持程序运行
# keyboard_listener.join()
mouse_listener.join()
