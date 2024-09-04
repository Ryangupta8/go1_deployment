from go1_deployment import DIR_PATH
from go1_deployment.source import Runner

if __name__ == "__main__":
    onnx_file = f"{DIR_PATH}/models/hist5_trot_model_25000.onnx"
    runner = Runner(onnx_file)
    try:
        runner.start_robot(init_duration=1, stance_duration=5)
        # runner.run()
    except (SystemExit, KeyboardInterrupt):
        runner.trigger_estop()
    runner.save_logs()

""" TODO
- fix imports
- tune PD gains (support per-joint gains)
"""
