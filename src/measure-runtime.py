import time


st = time.time()

for i in range(70):
    print('hi')

et = time.time()
runtime = et - st
print(runtime)
