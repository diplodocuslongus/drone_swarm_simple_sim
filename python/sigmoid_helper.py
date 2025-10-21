# helper to adjust parameters for cohesion metri c
import numpy as np
import matplotlib.pyplot as plt

# parameters from the simulation (in metrics.cpp)
s_scale = 3.0    # pre-scale multiplier (try 3..8)
k_sigmoid = 5.0  # steepness (try 3..10)
c0 = 0.7         # center/shift of sigmoid (try 0.0 or mean value)
alpha = 0.05     # exponential moving average smoothing 
k = 3.0

# for comparison
alphas = [0.01,0.05, 0.1, 0.3]
k_sigmoids = [3.0, 6.0, 9.]

# corresponding cpp version
# metric_smoothed_ = alpha * rawC + (1.0f - alpha) * metric_smoothed_

# not used
def moving_average_numpy(data, window_size):
    # The weights for a simple moving average are just 1/window_size
    weights = np.ones(window_size) / window_size
    # np.convolve does the rolling sum, 'valid' mode returns only full windows
    return np.convolve(data, weights, 'valid')

# EMA

def exponential_moving_average(data, alpha, init_value=None):
    if init_value is None:
        init_value = data[0]
    metric_smoothed = np.zeros_like(data)
    metric_smoothed[0] = init_value
    for i in range(1, len(data)):
        metric_smoothed[i] = alpha * data[i] + (1 - alpha) * metric_smoothed[i-1]
    return metric_smoothed

def sigmoid(x,k):
    return 1 / (1 + np.exp(-k*x))
# data = np.linspace(0.5,-0.5,nbel)
data = np.array([ 0.34961 ,  0.34684 ,  0.34628 ,  0.33469 ,  0.32648 ,  0.32789 ,  0.32791 ,  0.32637 ,  0.32315 ,  0.31760 ,  0.30901 ,  0.29644 ,  0.27912 ,  0.24629 ,  0.18614 ,  0.12409 ,  0.05491 ,  -0.02996 ,  -0.10672 ,  -0.19819 ,  -0.27893 ,  -0.34902 ,  -0.42680 ,  -0.51346 ,  -0.60819 ,  -0.71080 ,  -0.82221 ,  -0.93782 ,  -1.06309 ,  -1.19522 ,  -1.33929 ,  -1.49452 ,   -1.65931,   -1.83648])
nbel = len(data)# 100
metric_smoothed = np.zeros(nbel)  # 
metric_smoothed[0] = data[0]
for i in range(1, nbel):
    metric_smoothed[i] = alpha * data[i] + (1 - alpha) * metric_smoothed[i-1]

x = s_scale * metric_smoothed - c0
C_sigmoid = sigmoid(x,k_sigmoid)
# C_sigmoid = 1.0 / (1.0 + np.exp(-k_sigmoid * x));

# fig, ax = plt.subplots()
# ax.plot(data, C_sigmoid, label="C_sigm")
# ax.plot(data, metric_smoothed, label="smoothed")
#
# # Reverse the x-axis
# ax.invert_xaxis() 
# ax.set_title("sigmoid smoothed metric")
# ax.grid(True)  
# ax.grid(color='gray', linestyle='--', linewidth=0.5)
# ax.set_xlabel("raw metric")
# ax.set_ylabel("moving average sigmoid")
# ax.legend()
# plt.show()

fig, axes = plt.subplots(len(alphas), len(k_sigmoids), figsize=(12, 8))
for i, alpha in enumerate(alphas):
    metric_smoothed = exponential_moving_average(data, alpha)
    x = s_scale * metric_smoothed - c0
    for j, k in enumerate(k_sigmoids):
        metric_smoothed_sigmoid = sigmoid(x, k)
        # axes[i, j].plot(data, label='Original', alpha=0.3)
        axes[i, j].plot(data, metric_smoothed, label=f'EMA (α={alpha})')
        axes[i, j].plot(data, metric_smoothed_sigmoid, label=f'Sigmoid (k={k})')
        axes[i,j].invert_xaxis() 
        axes[i, j].grid(True)
        axes[i, j].grid(color='gray', linestyle='--', linewidth=0.5)
        axes[i, j].set_xlabel("raw metric")
        axes[i, j].set_title(f'α={alpha}, k={k}')
        axes[i, j].legend()
fig.suptitle(f'c0={c0}, s_scale={s_scale}', fontsize=16)
plt.tight_layout()
plt.show()
