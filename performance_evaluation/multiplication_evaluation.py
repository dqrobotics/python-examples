from dqrobotics import *
import time
import numpy as np
import scipy.io


def get_list_of_random_dq(size):
    list_of_dq = []
    for i in range(0, size):
        list_of_dq.append(DQ(np.random.rand(8, 1)))
    return list_of_dq


NUMBER_OF_RANDOM = 1000
NUMBER_OF_RUNS = 1000

time_vec = []
variance = []

for i in range(0, NUMBER_OF_RUNS):
    random_dq_a = get_list_of_random_dq(NUMBER_OF_RANDOM)
    random_dq_b = get_list_of_random_dq(NUMBER_OF_RANDOM)
    random_dq_c = get_list_of_random_dq(NUMBER_OF_RANDOM)
    start = time.time()
    for j in range(0, NUMBER_OF_RANDOM):
        random_dq_c[j] = random_dq_a[j] * random_dq_b[j]
    end = time.time()
    time_vec.append(end - start)
    variance.append(np.var(time_vec))

dictionary = {
    'time': time_vec,
    'variance': variance
}

scipy.io.savemat('multiplication_evaluation.mat', dictionary)
