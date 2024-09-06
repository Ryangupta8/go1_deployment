from go1_deployment import DIR_PATH
from go1_deployment.source import Runner

if __name__ == "__main__":
    onnx_file = f"{DIR_PATH}/models/trot03_model_99999.onnx"
    runner = Runner(onnx_file)
    try:
        runner.start_robot(init_duration=1, stance_duration=5)
        runner.run()
    except (SystemExit, KeyboardInterrupt):
        runner.trigger_estop()
    runner.save_logs()

""" TODO
- tune PD gains (support per-joint gains)
"""
