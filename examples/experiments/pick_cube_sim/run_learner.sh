export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python /home/agilex/chenjin/hil-serl-sim/examples/train_rlpd_sim.py "$@" \
    --exp_name=pick_cube_sim \
    --checkpoint_path=six_run \
    --demo_path=/home/agilex/chenjin/hil-serl-sim/demo_data/pick_cube_sim_30_demos_2025-02-17_15-56-01.pkl \
    --learner \
