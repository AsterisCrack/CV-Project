#This files executes the neccessary commands to remove the servo jitter
#It needs to execute commands on the terminal to work

import os
import time
import sys
import subprocess

#This function executes the command on the terminal
def execute_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    process.wait()
    return process.returncode

#This function removes the jitter from the servo
def remove_jitter():
    print("Executing up neccessary commands for servo jitter removal")
    #This command stops the pigpiod daemon
    success = execute_command("sudo killall pigpiod")
    time.sleep(1)
    #This command starts the pigpiod daemon
    execute_command("sudo pigpiod")
    time.sleep(1)
    print("Servo jitter removal complete")
