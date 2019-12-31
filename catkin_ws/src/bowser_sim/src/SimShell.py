#!/usr/bin/env python

import cmd, sys, time
import subprocess
try:
    from subprocess import DEVNULL # py3k
except ImportError:
    import os
    DEVNULL = open(os.devnull, 'wb')

from datetime import datetime
import rospy, roslaunch

from bowser_msg.msg import CommandVector

# TEXT COLORS for use with textWrap()
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

class SimShell(cmd.Cmd):

    intro   = 'simulation tool for bowser_sim'
    prompt  = '(Bowser)'
    file    = None  

    def do_end(self, arg):
        'End the sim shell'

        return True    

    def do_record(self, arg):
        'Record a topic\'s messages to a file'                

        file = topic = ""
        args = str.split(arg)

        if(len(args) == 0 or len(args) > 2):
            print(FAIL + 'Not enough args' + ENDC)
            return
        elif(len(args) == 1):
            topic = args[0]
            file = "record.txt"
        elif(len(args) == 2):
            topic = args[0]
            file = args[1] +".txt"

        print(OKBLUE + 'Writing messages to file : ' + file + ENDC)
        print(OKBLUE + 'Ctrl+C (SIGINT) to stop' + ENDC)

        # writes messages to file until SIGINT called
        with open(file, "w+") as output:
            subprocess.call(['rostopic','echo',args[0]], stdout=output)

        # open the recorded file, DEVNULL supresses annoying gedit errors    
        proc = subprocess.Popen(['gedit',file], stderr=DEVNULL)        
        


if __name__=='__main__':

    rospy.init_node('sim_shell', anonymous=True)
    SimShell().cmdloop('Interactive testing suite for bowser_sim')  