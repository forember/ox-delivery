import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

num_robots = np.array([1, 2, 4, 8, 16, 20, 32], dtype =np.float)

cave_cac = np.array([ 1, 2, 4, 8, 12, 13, 13], dtype=np.float)*100.0/num_robots
cave_crc = np.array([ 1, 2, 4, 8, 15, 19, 22], dtype=np.float)*100.0/num_robots
cave_nrc = np.array([ 1, 2, 3, 4, 5, 6, 7], dtype=np.float)*100.0/num_robots
cave_fhk = np.array([ 1, 2, 4, 8, 12, 13, 15 ], dtype=np.float)*100.0/num_robots

rural_crc =np.array([ 1, 2, 4, 8, 14, 18, 21], dtype=np.float)*100/num_robots
rural_cac =np.array([ 1, 2, 4, 7, 11, 12, 13 ], dtype=np.float)*100/num_robots
rural_nrc =np.array([ 1, 2, 3, 4, 5, 6, 8], dtype=np.float)*100/num_robots
rural_fhk =np.array([ 1, 2, 4, 8, 13, 15, 16 ], dtype=np.float)*100/num_robots

multi_cac =np.array([ 1, 2, 4, 8, 13, 13, 14 ], dtype=np.float)*100/num_robots
multi_crc =np.array([ 1, 2, 4, 8, 15, 19, 28], dtype=np.float)*100/num_robots
multi_nrc =np.array([ 1, 2, 3, 4, 5, 6, 7], dtype=np.float)*100/num_robots
multi_fhk =np.array([ 1, 2, 4, 8, 14, 17, 20 ], dtype=np.float)*100/num_robots

indoor_cac = np.array([ 1, 2, 4, 8, 15, 16, 17], dtype=np.float)*100/num_robots
indoor_crc = np.array([ 1, 2, 4, 8, 16, 18, 24 ], dtype=np.float)*100/num_robots
indoor_nrc = np.array([ 1, 2, 3, 4, 5, 6, 8], dtype=np.float)*100/num_robots
indoor_fhk = np.array([ 1, 2, 4, 8, 11, 14, 15 ], dtype=np.float)*100/num_robots

rural_crc = (rural_crc + indoor_crc + multi_crc + cave_crc)/4
rural_cac = (rural_cac + indoor_cac + multi_cac + cave_cac)/4
rural_nrc = (rural_nrc + indoor_nrc + multi_nrc + cave_nrc)/4
rural_fhk = (rural_fhk + indoor_fhk + multi_fhk + cave_fhk)/4

#plt.plot()
##plt.set_title("", fontweight='bold')
#plt.plot(num_robots,rural_crc,'b', label="CRC")
#plt.plot(num_robots,rural_cac, 'g', label="CAC")
#plt.plot(num_robots,rural_nrc, 'r',label="NRC")
#plt.plot(num_robots,rural_fhk, 'm',label="FHK")
#plt.legend(loc=3)
#plt.plot(num_robots,rural_crc, 'b+', label="CRC")
#plt.plot(num_robots,rural_cac, 'g+',label="CAC")
#plt.plot(num_robots,rural_nrc, 'r*',label="NRC")
#plt.plot(num_robots,rural_fhk, 'm*',label="NRC")
#plt.ylabel('Utilization (%)', fontsize = 15, fontweight='bold')
#plt.ylim([0,120])
#plt.xlabel('Number of Robots', fontweight='bold')
#plt.show()

plt.plot()
width = 0.2 

plt.set_title("Cave Environment", fontweight='bold')
#axarr[0,0].plot(num_robots,cave_crc, label="CRC")
#axarr[0,0].plot(num_robots,cave_cac, label="CAC")
#axarr[0,0].plot(num_robots,cave_nrc, label="NRC")
#axarr[0,0].plot(num_robots,cave_fhk, label="FHK")
ind = np.arange(len(num_robots))
b1=plt.bar(ind, cave_crc, width, color='b')#,  label="CRC")
b2=plt.bar(ind+width,cave_cac, width, color ='g')# label="CAC")
b3=plt.bar(ind+2.0*width,cave_nrc, width, color='r')# label="NRC")
b4=plt.bar(ind+3.0*width,cave_fhk, width, color='c')
plt.legend([b1,b2,b3,b4],["CRC", "CAC","NRC", "FHK"],loc=1)
#axarr[0,0].plot(num_robots,cave_crc, 'g+', label="CRC")
#axarr[0,0].plot(num_robots,cave_cac, 'b+',label="CAC")
#axarr[0,0].plot(num_robots,cave_nrc, 'r*',label="NRC")
plt.set_ylabel('Utilization (%)', fontweight='bold')
plt.set_xticks((ind+2*width))
plt.set_xticklabels(num_robots)
#vals = axarr[0,0].get_yticks()
#axarr[0,0].set_yticklabels(['{:3.2f}%'.format(x*100) for x in vals])
#axarr[0,0].yaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda y, _: '{:.0}'.format(y)))
plt.set_ylim([0,120])
#axarr[0,0].yaxis.set_major_formatter(mpl.ticker.ScalarFormatter(useMathText=True, useOffset=False))
plt.set_xlabel('Number of Robots')
plt.show()
