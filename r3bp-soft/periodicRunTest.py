from imutils.video.pivideostream import PiVideoStream
import imutils
from periodicrun import periodicrun
import time

camera = PiVideoStream(resolution=(640,480),framerate=90).start()
time.sleep(2.0)

def exampleLoop(arg1, arg2):
    print(arg1, arg2, "Execution time (microseconds): ", int(round( time.time()*1000000 )))

testArg1 = "Test arg"
testArg2 = 999.999
pr = periodicrun(0.01, exampleLoop, (testArg1, testArg2,), 0.05, 0.01)
pr.run_thread()
pr.join()
camera.stop()