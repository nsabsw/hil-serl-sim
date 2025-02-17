import pickle

import debugpy
try:
    # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
    debugpy.listen(("localhost", 9501))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()
except Exception as e:
    pass


# 定义要打开的 .pkl 文件路径
file_path = '/home/agilex/chenjin/hil-serl-sim/classifier_data/pick_cube_sim_200_success_images_2025-02-14_19-41-57.pkl'

try:
    # 以二进制读取模式打开 .pkl 文件
    with open(file_path, 'rb') as file:
        # 使用 pickle.load() 方法加载文件中的对象
        data = pickle.load(file)
    print("文件读取成功！")
    # 打印读取到的数据
    print(data)
except FileNotFoundError:
    print(f"未找到文件：{file_path}")
except pickle.UnpicklingError:
    print("在反序列化过程中出现错误，文件可能已损坏。")
except Exception as e:
    print(f"发生未知错误：{e}")