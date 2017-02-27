import sys
import os


test_images = ["office.png", "cave.png", "greg_cottage.png", "multicell.png"]
alg_modes = ["0"]#, "1", "2"]

output_file = open("all_results.txt", "w")

for alg_mode in alg_modes:
    for img in test_images:
        print "p"
        os.system("./UseLibs/test UseLibs/test_data/ " + img + " 2 " + alg_mode)

#list of files in the current directory
list_of_files = os.listdir(os.getcwd())
output_file.write("------------------------------CRC--------------------------------\n")
for each_file in list_of_files:
    #since its all type str you can simply use startswith
    if each_file.startswith('test_CRC'):  
        curr_file = open(each_file)
        output_file.write(curr_file.read())
'''
output_file.write("------------------------------CAC--------------------------------\n")
for each_file in list_of_files:
    #since its all type str you can simply use startswith
    if each_file.startswith('test_CAC'):  
        curr_file = open(each_file)
        output_file.write(curr_file.read())
output_file.write("------------------------------NRC--------------------------------\n")
for each_file in list_of_files:
    #since its all type str you can simply use startswith
    if each_file.startswith('test_NRC'):  
        curr_file = open(each_file)
        output_file.write(curr_file.read())
'''
