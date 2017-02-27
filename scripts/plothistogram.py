import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

num_robots = [1, 2, 4, 8, 16, 20, 32]

#Tests are run8 ning on image ------ Compare_1.png
#world_cac = [584812, 310716, 176186, 102556, 71648.9, 69285.3, 69285.3]
cave_cac = [ 584812, 310716, 176186, 102556, 71648.9, 69285.3, 69285.3]
cave_crc = [ 584812, 301710, 171940, 113390, 70143, 70143, 70143  ]
cave_nrc = [ 584812, 328146, 320470, 246992, 217055, 172995, 151322  ] 

rural_crc = [ 555047, 287182,164908 ,112412 ,73271.5 ,73271.5 ,73271.5]
rural_cac = [ 555047, 253026,191208 ,101289 ,73584.1 ,73584.1 ,73584.1]
rural_nrc = [ 555047, 314423, 319083, 268902, 195281, 173921, 154559]

multi_cac = [586826,330202 , 183101 ,108773 ,74898.5 , 74898.5 ,74898.5 ]
multi_crc = [586826,294272 , 167081 ,96581.9, 83906.3, 83906.3 ,72917.5]
multi_nrc = [586826,323445 , 330780 ,254111 ,181035,    179478 ,154373 ]

indoor_cac = [223337 ,116526 ,68086.7 ,35865   ,20191.7 ,18662.4 ,18662.4 ]
indoor_crc = [223337 ,119387 ,61290.1 ,34554.6 ,22726.1 ,21008.1 ,18498.7 ]
indoor_nrc = [223337, 121732, 117299 ,88080.4 ,74896.3 ,63780.2 ,59776.5]

f, axarr = plt.subplots(2, 2)

axarr[0,0].set_title("Cave Environment", fontweight='bold')
axarr[0,0].plot(num_robots,cave_crc,  'g',label="CRC")
axarr[0,0].plot(num_robots,cave_cac, 'b',label="CAC")
axarr[0,0].plot(num_robots,cave_nrc, 'r',label="NRC")
axarr[0,0].legend(loc=1)
axarr[0,0].plot(num_robots,cave_crc, 'g+', label="CRC")
axarr[0,0].plot(num_robots,cave_cac, 'b+',label="CAC")
axarr[0,0].plot(num_robots,cave_nrc, 'r*',label="NRC")
axarr[0,0].set_ylabel('Maximum Area Covered (m)', fontweight='bold')
axarr[0,0].yaxis.set_major_formatter(mpl.ticker.ScalarFormatter(useMathText=True, useOffset=False))
#axarr[0,0].set_xlabel('Number of Robots')


axarr[0,1].set_title("Rural Quebec", fontweight='bold')
axarr[0,1].plot(num_robots,rural_crc, label="CRC")
axarr[0,1].plot(num_robots,rural_cac, label="CAC")
axarr[0,1].plot(num_robots,rural_nrc, label="NRC")
#axarr[0,1].set_ylabel('Maximum Area Covered (m)', fontweight='bold')
#axarr[0,1].set_xlabel('Number of Robots')
axarr[0,1].legend(loc=1)
axarr[0,1].plot(num_robots,rural_crc, 'g+',label="CRC")
axarr[0,1].plot(num_robots,rural_cac, 'b+',label="CAC")
axarr[0,1].plot(num_robots,rural_nrc, 'r*',label="NRC")

axarr[1,0].set_title("Multi-cell Environment", fontweight='bold')
axarr[1,0].plot(num_robots,multi_crc, label="CRC")
axarr[1,0].plot(num_robots,multi_cac, label="CAC")
axarr[1,0].plot(num_robots,multi_nrc, label="NRC")
axarr[1,0].set_ylabel('Maximum Area Covered(m)', fontweight='bold')
axarr[1,0].set_xlabel('Number of Robots', fontweight='bold')
axarr[1,0].legend(loc=1)
axarr[1,0].plot(num_robots,multi_crc, 'g+',label="CRC")
axarr[1,0].plot(num_robots,multi_cac, 'b+',label="CAC")
axarr[1,0].plot(num_robots,multi_nrc, 'r*',label="NRC")

axarr[1,1].set_title("Indoor Environment", fontweight='bold')
axarr[1,1].plot(num_robots,indoor_crc, label="CRC")
axarr[1,1].plot(num_robots,indoor_cac, label="CAC")
axarr[1,1].plot(num_robots,indoor_nrc, label="NRC")
#axarr[1,1].set_ylabel('Maximum Area Covered', fontweight='bold')
axarr[1,1].set_xlabel('Number of Robots', fontweight='bold')
axarr[1,1].legend(loc=1)
axarr[1,1].plot(num_robots,indoor_crc, 'g+',label="CRC")
axarr[1,1].plot(num_robots,indoor_cac, 'b+',label="CAC")
axarr[1,1].plot(num_robots,indoor_nrc, 'r*',label="NRC")

plt.show()
