import math

def normalized_theta(x):
    x = x + math.pi
    x = x - math.pi * math.floor(x/2/math.pi)
    x = x - math.pi
    return x

for i in range(0,100):
    x = (i-50) * 0.05
    print(x,normalized_theta(x))