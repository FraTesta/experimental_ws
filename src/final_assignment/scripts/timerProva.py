import sys 
import os 
import time
import threading

class lol:
    def __init__(self):
        self.stop = False
    
    def cancel(self):
        print('goal cancelled')
        self.stop = True        
        return self.stop

if __name__ == '__main__':
    cl = lol()
    timer = threading.Timer(5.0, cl.cancel)
    timer.start()
    while not cl.stop:
        print('Navigation')
        time.sleep(1)
