#generate sine wave table for 8 bit PWM.(space vector modulation)
#file path: Core/Inc/table.h (c lang file)
import math

######settings######

wave_size = 256 #table size
##attention: table size is 5/3 of wave size.##
table_size = int(wave_size*5/3)+1
U_point = 0
V_point = int(wave_size/3)
W_point = int(wave_size*2/3)


max = 65535 #max value

######settings end######


def wave_function(x,period):
    return max/2 + max/2*(math.sin(x*2*math.pi/period)+1/6*math.sin(3*x*2*math.pi/period))


table = [wave_function(x,wave_size) for x in range(table_size)]



###test###
# import matplotlib.pyplot as plt
# plt.plot(table)
# plt.show()
###test end###


######save file######

text = """
#ifndef __TABLE_H__
#define __TABLE_H__

#include "stdint.h"

"""

text +="#define TABLE_SIZE "+ str(table_size)+"\r"
text +="#define WAVE_SIZE "+ str(wave_size)+"\r"
text +="#define U_POINT "+ str(U_point)+"\r"
text +="#define V_POINT "+ str(V_point)+"\r"
text +="#define W_POINT "+ str(W_point)+"\r"

text +="""
const uint16_t sine_table["""+str(table_size)+"""] = {\r"""
count = 0
for i in range(table_size):
    count += 1
    if count == 30:
        count = 0
        text += "\r"
    text += str(int(table[i])).rjust(3)
    if i != table_size-1:
        text += ","
text +="};"

text +="""



#endif
"""
print(text)


save_file = open("Core/Inc/table.h", "w")
save_file.write(text)
save_file.close()







