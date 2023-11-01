from setuptools import setup
from Cython.Build import cythonize

# setup(
#     ext_modules=cythonize("testcython.py"),
# )

setup(
    ext_modules=cythonize(
        ['testcython2.pyx',                  # Cython code file with primes() function
         'testcython.py',
         'cythonUtils.pyx'],                     # Python code file with primes() function
        annotate=True),                 # enables generation of the html annotation file
)


