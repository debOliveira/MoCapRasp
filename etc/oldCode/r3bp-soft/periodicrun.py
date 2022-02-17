'''
MIT License

Copyright (c) 2016 Burak Kakillioğlu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
'''
Author: Burak Kakillioglu
Date: 11/12/2016
Library name: periodicrun
Description:
Python library for running a loop periodically with ~100 microseconds accuracy.
This library provides functionality for periodic execution of a method with accurate timing.
It is meant by accurate timing that periodicrun guaranties that the loop will run every x seconds with 100 microsends (default) accuracy.
periodicrun uses Python's built-in sched library to schedule the execution of the given method periodically.
'''

import sched, time, threading

class periodicrun:

    delta = 0.0
    cycleCount = 0
    is_thread = False

    def __init__(self, period, loop, args=(), lifetime=0, accuracy=0.0001):
        self.s = sched.scheduler(time.time, time.sleep)
        self.period_acc = float(period)*accuracy
        self.loop = loop
        self.args = list(args)
        self.lifetime = float(lifetime)
        self.sample = 1/accuracy
        self.tolerance = period * 1.1

    def run(self):
        self.offset = time.time()
        self.lasttime = self.offset
        self.cycle()
        self.start_loop()

    def run_thread(self):
        self.is_thread = True
        self.t = threading.Thread(target=self.start_loop)
        self.offset = time.time()
        self.lasttime = self.offset
        self.cycle()
        self.t.start()

    def start_loop(self):
        self.s.run()

    def cycle(self):
        self.delta = self.delta + self.period_acc
        self.cycleCount = self.cycleCount+1
        self.s.enterabs(self.offset + self.delta, 1, self.outer_loop, ())

    def outer_loop(self):
        if self.lifetime == 0 or self.delta < self.lifetime:
            self.cycle()

        if self.cycleCount == self.sample:
            self.loop(*self.args)
            self.cycleCount = 0

            # Check if execution time exceeds period duration with %10 tolerance
            now = time.time()
            #print "now: ", now, ". self.lasttime: ", self.lasttime, ". diff: ", now-self.lasttime, "tol: ", self.tolerance
            if now-self.lasttime > self.tolerance:
                print("Warning: Execution time exceeds period duration")
            self.lasttime = now

    def join(self):
        if self.is_thread:
            self.t.join()
        else: print("Error: Periodic run is not a seperate thread.")

    def interrupt(self):
        if self.is_thread:
            self.lifetime = 0.000001
            print("Loop interrupted")
        else: print("Error: Periodic run is not a seperate thread.")
