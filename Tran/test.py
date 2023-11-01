import testcython
import testcython2
from utils import *
import time
import math


start1 = time.time()
testcython.test_non_cython()
end1 = time.time()
time1 = end1 - start1 

start3 = time.time()
testcython2.test_non_cython()
end3 = time.time()
time3 = end3 - start3 

start2 = time.time()
i = 0
nb_primes = 5000000
cnt = 0
while i < nb_primes:
        n = int (math.sqrt(i))
        i += 1
        cnt += n
        
print(cnt)
end2 = time.time()
time2 = end2 - start2 


print(time1, time3, time2)