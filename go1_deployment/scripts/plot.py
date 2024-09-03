from go1_deployment.source import Logger, read_pickled_data, DIR_PATH

log = Logger()
read_pickled_data(log, f"{DIR_PATH}/logs/Aug-30-2024_1415.pickle")
