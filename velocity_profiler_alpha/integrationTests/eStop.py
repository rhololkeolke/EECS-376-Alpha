#!/usr/bin/env python
'''
Created on Apr 1, 2012

@author: Devin Schwab
'''

# toggle eStop when space is pressed

# Standard ros commands to make a node
import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from std_msgs.msg._Bool import Bool as BoolMsg

import curses
import threading
from Queue import Queue

class Publisher(threading.Thread):
    def __init__(self,publisher,rate):
        threading.Thread.__init__(self)
        self.publisher = publisher
        self.result = None
        self.char = Queue()
        self.enabled = True
        self.naptime = rospy.Rate(rate)
        
    def get_result(self):
        return self.result
    
    def run(self):
        while True:
            if(self.char.qsize() > 0):
                char = self.char.get() # get the next character in the queue
            else:
                char = 0
            
            if(char == 32):
                self.enabled = not self.enabled
            elif(char == 113):
                break
            
            msg = BoolMsg()
            msg.data = self.enabled
            self.publisher.publish(msg)
            self.naptime.sleep()
            

def eStop():
    rospy.init_node('estopSim')
    cmdVelPub = rospy.Publisher('motors_enabled',BoolMsg)     
    
    try:
        myscreen = curses.initscr()
        myscreen.border(0)
        myscreen.addstr(12,25,"Press Space to toggle eStop, press q to quit")
        curses.cbreak()
        curses.noecho()
        myscreen.refresh()
    except:
        curses.nocbreak()
        curses.echo()
        curses.endwin()
        
    pubThread = Publisher(cmdVelPub,20.0)
    pubThread.start()

    sleepTime = rospy.Duration(1)

    while 1:
        char = myscreen.getch()
        
        pubThread.char.put(char,True)
        
        if(char == 113):
            pubThread.join(1000)
            break
        
        rospy.sleep(sleepTime)
    
    curses.nocbreak()
    curses.echo()
    curses.endwin()

if __name__ == "__main__":
    eStop()