import copy
import os
from tqdm import tqdm
import numpy as np
import pickle as pkl
import datetime
from absl import app, flags
from pynput import keyboard

from experiments.mappings import CONFIG_MAPPING

from franka_sim.utils.viewer_utils import DualMujocoViewer

# import debugpy
# try:
#     # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
#     debugpy.listen(("localhost", 9501))
#     print("Waiting for debugger attach")
#     debugpy.wait_for_client()
# except Exception as e:
#     pass

FLAGS = flags.FLAGS
flags.DEFINE_string("exp_name", None, "Name of experiment corresponding to folder.")
flags.DEFINE_integer("successes_needed", 200, "Number of successful transistions to collect.")


success_key = False
start_key = False
def on_press(key):
    global success_key
    try:
        if str(key) == 'Key.enter':
            success_key = True
    except AttributeError:
        pass

def main(_):
    global success_key
    listener = keyboard.Listener(
        on_press=on_press)
    listener.start()
    assert FLAGS.exp_name in CONFIG_MAPPING, 'Experiment folder not found.'
    config = CONFIG_MAPPING[FLAGS.exp_name]()
    env = config.get_environment(fake_env=False, save_video=False, classifier=False)

    obs, _ = env.reset()
    successes = []
    failures = []
    success_needed = FLAGS.successes_needed
    pbar = tqdm(total=success_needed)
    
    # Create the dual viewer
    dual_viewer = DualMujocoViewer(env.unwrapped.model, env.unwrapped.data)

    print("Press shift to start recording, press enter to record a successful transition.\nIf your controller is not working check controller_type (default is xbox) is configured in examples/experiments/pick_cube_sim/config.py")
    with dual_viewer as viewer:

        while viewer.is_running():
            
            actions = np.zeros(env.action_space.sample().shape) 
            next_obs, rew, done, truncated, info = env.step(actions)
            viewer.sync()
            if "intervene_action" in info:
                actions = info["intervene_action"]

            transition = copy.deepcopy(
                dict(
                    observations=obs,
                    actions=actions,
                    next_observations=next_obs,
                    rewards=rew,
                    masks=1.0 - done,
                    dones=done,
                )
            )
            obs = next_obs
            if success_key or info["succeed"]:
                successes.append(transition)
                pbar.update(1)
                success_key = False
            else:
                failures.append(transition)

            if done or truncated:
                obs, _ = env.reset()
            if len(successes) >= success_needed:
                break

    if not os.path.exists("./classifier_data"):
        os.makedirs("./classifier_data")
    uuid = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"./classifier_data/{FLAGS.exp_name}_{success_needed}_success_images_{uuid}.pkl"
    with open(file_name, "wb") as f:
        pkl.dump(successes, f)
        print(f"saved {success_needed} successful transitions to {file_name}")

    file_name = f"./classifier_data/{FLAGS.exp_name}_failure_images_{uuid}.pkl"
    with open(file_name, "wb") as f:
        pkl.dump(failures, f)
        print(f"saved {len(failures)} failure transitions to {file_name}")
        
if __name__ == "__main__":
    app.run(main)
