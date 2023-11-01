import cython
import math 
from utils import *

def test_non_cython():
    # a = 5
    # while a < 10:
    #     a += 1
    # print(math.sqrt(a), primes(a))
    print(primes(5000000))

def primes(nb_primes: cython.int):
    i: cython.int = 0
    # p: cython.int[1000]

    # if nb_primes > 1000:
    #     nb_primes = 1000

    # if not cython.compiled:  # Only if regular Python is running
    #     p = [0] * 1000       # Make p work almost like a C array

    # len_p: cython.int = 0  # The current number of elements in p.
    # n: cython.int = 2
    # while len_p < nb_primes:
    #     # Is n prime?
    #     for i in p[:len_p]:
    #         if n % i == 0:
    #             break

    #     # If no break occurred in the loop, we have a prime.
    #     else:
    #         p[len_p] = n
    #         len_p += 1
    #     n += 1

    # # Let's copy the result into a Python list:
    # result_as_list = [prime for prime in p[:len_p]]
    
    # i = Utils.distanceBetweenTwoPoints(0, 0, 2, 0)
    n: cython.int
    cnt: cython.int = 0
    while i < nb_primes:
        n = int (math.sqrt(i))
        i += 1
        cnt += n
    return cnt