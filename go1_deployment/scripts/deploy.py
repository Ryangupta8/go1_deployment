from go1_deployment import DIR_PATH
from go1_deployment.source import Runner

if __name__ == "__main__":
    onnx_file = f"{DIR_PATH}/models/hist5_trot_model_25000.onnx"
    runner = Runner(onnx_file)
    try:
        runner.start_robot(init_duration=1, stance_duration=5)
        runner.run()
    except (SystemExit, KeyboardInterrupt):
        runner.trigger_estop()
    runner.save_logs()

""" TODO
- refactor repo as:
go1_deployment
    go1_deployment
        __init__
        source
            __init__
            constants
            control_loop
            runner
            logger
            utils
        scripts
            __init__
            deploy
            plot
        logs
            __init__
        third_party
            __init__
        models
            __init__
- delete dead code
- fix imports
- remove unneeded try/except (see init procedure)
- tune PD gains (support per-joint gains)
"""
