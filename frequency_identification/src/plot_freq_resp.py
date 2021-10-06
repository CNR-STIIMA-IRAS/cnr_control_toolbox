#!/usr/bin/env python3
# license removed for brevity

import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys

if __name__ == '__main__':
    
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) < 2:
        print("usage: rosrun frequency_response plot_freq_resp.py namespace")
        raise ValueError("usage: rosrun frequency_response plot_freq_resp.py namespace")
    
    

    rospy.init_node('plot_freq_resp', anonymous=True)
    name=argv[1]
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
    
    a4_dims = (11.7, 8.27)
    f, (ax1, ax2) = plt.subplots(2,1, sharex=True,figsize=a4_dims);
    ax1.semilogx(w, magnitude)    # Bode magnitude plot
    ax1.set_xlabel("Angular frequency [rad/s]")
    ax1.set_ylabel("Magnitude")
    ax1.grid(axis="both",which="both")
    
    ax2.semilogx(w, phase)  # Bode phase plot
    ax2.set_xlabel("Angular frequency [rad/s]")
    ax2.set_ylabel("Phase [deg]")
    ax2.grid(axis="both",which="both")
    plt.show(block = False)
    plt.pause(0.01)
    rospy.spin()