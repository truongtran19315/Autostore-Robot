import time

i = 0

begin = time.time()
while True:
    i = i + 1
    if i == 360:
        break

end = time.time()

timee = end - begin 

print ("elapsed_time:{0}".format(timee) + "[sec]")