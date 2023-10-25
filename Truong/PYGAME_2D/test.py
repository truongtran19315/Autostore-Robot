import time
import math


def test(): 
    i = 0
    b = 0
    while True:
        i = i + 1
        if i == 24:
            break
        
        b = math.sqrt((3453-3434)**2 + (3434-2332)**2)  
    
    while True:
        i = i + 1
        if i == 360:
            break
        
        j = 0
        while True:
            j = j + 1
            if j == 24:
                break
            a = math.sin(i/360)
            c = math.cos(i/360)
            b = math.sqrt(500**2 + 700**2)
            b = math.sqrt(434**2 + 234**2)
            
            b = math.sqrt(243**2 + 453**2)
            
            b = math.sqrt(543**2 + 542**2)
            
    return b
        

begin = time.time()

test()

end = time.time()

timee = end - begin 
times = timee*1000

print ("elapsed_time:{0}".format(times) + "[milisec]")