from timer import Timer
import time
timer = Timer()

i = 1000000000
while i > 0:
    i -= 1
    timer.tick()
    if timer.elapsed_seconds_since_lap() >= 1:
        print(timer.ticks_per_second())
        timer.lap()
        print('lap')
    #print(i)
