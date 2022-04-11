import time

timeout = 10
first_time = time.time()
last_time = first_time
while(True):
    pass #do something here
    new_time = time.time()
    if  new_time - last_time > timeout:
        last_time = new_time
        print("Its been %f seconds" % (new_time - first_time))