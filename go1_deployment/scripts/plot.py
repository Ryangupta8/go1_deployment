from go1_deployment.source import DIR_PATH, Logger, read_pickled_data

log = Logger()
read_pickled_data(log, f"{DIR_PATH}/logs/Aug-30-2024_1415.pickle")
