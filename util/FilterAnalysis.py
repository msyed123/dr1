#py3 only due to pip with matplotlib

import numpy as np
import matplotlib.pyplot as plt
import json
from scipy import signal 

x_axis = []
y_axis = []
z_axis = []
x0_axis = []
y0_axis = []
z0_axis = []
time_stamp = []
length = 0

#accessing txt doc
f = open("output.txt", 'r')
for line in f:
    new_line = json.loads(line)
    x_axis.append(new_line[0])
    y_axis.append(new_line[1])
    z_axis.append(new_line[2])
    x0_axis.append(new_line[3])
    y0_axis.append(new_line[4])
    z0_axis.append(new_line[5])
    time_stamp.append(new_line[6])
    length += 1

f.close()

#######################################3
#Filter
order = 5
sampling_freq = 20
cutoff_freq = 2
sampling_duration = 5

normalized_cutoff_freq = 2*cutoff_freq/sampling_freq

numerator_coeff, denominator_coeff = signal.butter(sampling_duration, normalized_cutoff_freq)

#sos = signal.ellip(2, .5, 10, 0.5, output='sos')
#filtered_x = signal.sosfilt(sos, x_axis)

filtered_x = signal.lfilter(numerator_coeff, denominator_coeff, x_axis)
filtered_y = signal.lfilter(numerator_coeff, denominator_coeff, y_axis)
filtered_z = signal.lfilter(numerator_coeff, denominator_coeff, z_axis)
filtered_x0 =signal.lfilter(numerator_coeff, denominator_coeff, x0_axis)
filtered_y0 =signal.lfilter(numerator_coeff, denominator_coeff, y0_axis)
filtered_z0 =signal.lfilter(numerator_coeff, denominator_coeff, z0_axis)



#plotting code
plt.figure(1)
plt.subplot(311)
plt.ylim(.0535, .056)
plt.scatter(time_stamp, x_axis, 2)
plt.plot(time_stamp, filtered_x)
plt.tight_layout()
plt.subplot(312)
plt.ylim(.031, .032)
plt.scatter(time_stamp, y_axis, 2)
plt.plot(time_stamp, filtered_y)
plt.tight_layout()
plt.subplot(313)
plt.ylim(-1.845, -1.825)
plt.scatter(time_stamp, z_axis, 2)
plt.plot(time_stamp, filtered_z)
plt.tight_layout()


plt.figure(2)
plt.subplot(311)
plt.ylim(.04, .06)
plt.scatter(time_stamp, x0_axis, 2)
plt.plot(time_stamp, filtered_x0)
plt.subplot(312)
plt.ylim(-.13, -.105)
plt.scatter(time_stamp, y0_axis, 2)
plt.plot(time_stamp, filtered_y0)
plt.subplot(313)
plt.ylim(-.033, -.045)
plt.scatter(time_stamp, z0_axis, 2)
plt.plot(time_stamp, filtered_z0)

plt.tight_layout()
plt.show()

