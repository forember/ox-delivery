import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

num_robots = np.array([1, 2, 4, 8, 16, 20, 32], dtype =np.float)

#Tests are run8 ning on image ------ Compare_1.png
#world_cac = [584812, 310716, 176186, 102556, 71648.9, 69285.3, 69285.3]
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

print "The utilizations will be\n" 
print "---------------CRC----------------------\n" 
print "Cave: ", cave_crc
print "Multicell: ", multi_crc
print "Rural: ", rural_crc
print "Indoor: ", indoor_crc
print "---------------CAC----------------------\n" 
print "Cave: ", cave_cac
print "Multicell: ", multi_cac
print "Rural: ", rural_cac
print "Indoor: ", indoor_cac
print "---------------FHK----------------------\n" 
print "Cave: ", cave_fhk
print "Multicell: ", multi_fhk
print "Rural: ", rural_fhk
print "Indoor: ", indoor_fhk
print "---------------NRC----------------------\n" 
print "Cave: ", cave_nrc
print "Multicell: ", multi_nrc
print "Rural: ", rural_nrc
print "Indoor: ", indoor_nrc


f, axarr = plt.subplots(2, 2)

axarr[0,0].set_title("Cave Environment", fontweight='bold')
axarr[0,0].plot(num_robots,cave_crc, label="CRC")
axarr[0,0].plot(num_robots,cave_cac, label="CAC")
axarr[0,0].plot(num_robots,cave_nrc, label="NRC")
axarr[0,0].plot(num_robots,cave_fhk, label="FHK")
#axarr[0,0].hist(num_robots,weights = cave_crc,  label="CRC")
#axarr[0,0].hist(num_robots,weights =cave_cac, label="CAC")
#axarr[0,0].hist(num_robots,weights =cave_nrc, label="NRC")
#axarr[0,0].hist(num_robots,weights =cave_fhk, label="FHK")
#axarr[0,0].legend(loc=3)
axarr[0,0].plot(num_robots,cave_crc, 'g+', label="CRC")
axarr[0,0].plot(num_robots,cave_cac, 'b+',label="CAC")
axarr[0,0].plot(num_robots,cave_nrc, 'r*',label="NRC")
axarr[0,0].set_ylabel('Utilization (%)', fontweight='bold')
#vals = axarr[0,0].get_yticks()
#axarr[0,0].set_yticklabels(['{:3.2f}%'.format(x*100) for x in vals])
#axarr[0,0].yaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda y, _: '{:.0}'.format(y)))
axarr[0,0].set_ylim([0,120])
#axarr[0,0].yaxis.set_major_formatter(mpl.ticker.ScalarFormatter(useMathText=True, useOffset=False))
axarr[0,0].set_xlabel('Number of Robots')


axarr[0,1].set_title("Rural Quebec", fontweight='bold')
axarr[0,1].plot(num_robots,rural_crc, label="CRC")
axarr[0,1].plot(num_robots,rural_cac, label="CAC")
axarr[0,1].plot(num_robots,rural_nrc, label="NRC")
axarr[0,1].plot(num_robots,rural_fhk, label="FHK")
#axarr[0,1].set_ylabel('Maximum Area Covered (m)', fontweight='bold')
#axarr[0,1].set_xlabel('Number of Robots')
#axarr[0,1].legend(loc=3)
axarr[0,1].plot(num_robots,rural_crc, 'g+',label="CRC")
axarr[0,1].plot(num_robots,rural_cac, 'b+',label="CAC")
axarr[0,1].plot(num_robots,rural_nrc, 'r*',label="NRC")
axarr[0,1].set_ylim([0,120])
#vals = axarr[0,1].get_yticks()
#axarr[0,1].set_yticklabels(['{:3.2f}%'.format(x*100) for x in vals])
#axarr[0,1].yaxis.set_major_formatter(mpl.ticker.FuncFormatter(lambda y, _: '{:.0%}'.format(y)))

axarr[1,0].set_title("Multi-cell Environment", fontweight='bold')
axarr[1,0].plot(num_robots,multi_crc, label="CRC")
axarr[1,0].plot(num_robots,multi_cac, label="CAC")
axarr[1,0].plot(num_robots,multi_nrc, label="NRC")
axarr[1,0].plot(num_robots,multi_fhk, label="FHK")
axarr[1,0].set_ylabel('Utilization (%)', fontweight='bold')
axarr[1,0].set_xlabel('Number of Robots', fontweight='bold')
#axarr[1,0].legend(loc=3)
axarr[1,0].plot(num_robots,multi_crc, 'g+',label="CRC")
axarr[1,0].plot(num_robots,multi_cac, 'b+',label="CAC")
axarr[1,0].plot(num_robots,multi_nrc, 'r*',label="NRC")
axarr[1,0].set_ylim([0,120])

axarr[1,1].set_title("Indoor Environment", fontweight='bold')
axarr[1,1].plot(num_robots,indoor_crc, label="CRC")
axarr[1,1].plot(num_robots,indoor_cac, label="CAC")
axarr[1,1].plot(num_robots,indoor_nrc, label="NRC")
axarr[1,1].plot(num_robots,indoor_fhk, label="FHK")
#axarr[1,1].set_ylabel('Maximum Area Covered', fontweight='bold')
axarr[1,1].set_xlabel('Number of Robots', fontweight='bold')
axarr[1,1].legend(loc=3)
axarr[1,1].plot(num_robots,indoor_crc, 'g+',label="CRC")
axarr[1,1].plot(num_robots,indoor_cac, 'b+',label="CAC")
axarr[1,1].plot(num_robots,indoor_nrc, 'r*',label="NRC")
axarr[1,1].set_ylim([0,120])

plt.show()
