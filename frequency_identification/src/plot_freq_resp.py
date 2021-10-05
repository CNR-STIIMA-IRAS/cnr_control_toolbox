#!/usr/bin/env python3
# license removed for brevity

import rospy
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    rospy.init_node('plot_freq_resp', anonymous=True)
    name="frequency_identification_test"
    fr_real=rospy.get_param(name+"/frequency_response/real")    
    fr_imag=rospy.get_param(name+"/frequency_response/imag")
    angular_frequency=rospy.get_param(name+"/frequency_response/angular_frequency")
    
    fr = np.array(fr_real, dtype=complex)
    fr.imag=np.array(fr_imag)
    w=np.array(angular_frequency)

    p = w.argsort()
    w=w[p]
    fr=fr[p]

    magnitude=np.abs(fr)
    phase=np.rad2deg(np.unwrap(np.angle(fr)))
    
    plt.figure()
    plt.semilogx(w, magnitude)    # Bode magnitude plot
    plt.xlabel("Angular frequency [rad/s]")
    plt.ylabel("Magnitude")
    plt.grid(axis="both",which="both")
    
    plt.figure()
    plt.semilogx(w, phase)  # Bode phase plot
    plt.xlabel("Angular frequency [rad/s]")
    plt.ylabel("Phase [deg]")
    plt.grid(axis="both",which="both")
    plt.show()