from go1_deployment import DIR_PATH
from go1_deployment.source import Logger, read_pickled_data

log = Logger()
# read_pickled_data(log, f"{DIR_PATH}/logs/Aug-30-2024_1415.pickle")
# read_pickled_data(log, f"{DIR_PATH}/logs/Sep-06-2024_1448.pickle")
read_pickled_data(log, f"{DIR_PATH}/logs/Sep-09-2024_1153.pickle")
