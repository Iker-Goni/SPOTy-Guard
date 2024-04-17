import threading
import time

stopcondition = False


def runforever():
    while stopcondition == False:
        print(time.time())


t1 = threading.Thread(target=runforever, name="Thread 1")
t1.start()

time.sleep(3)
stopcondition = True
print("stopped")

t1.join()

