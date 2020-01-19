#!/usr/bin/python
"""
    Test script to verify use of a control file to signal to
    a running Python process.
    to terminate issue "echo 1 > control" at console
"""

# initialize control file and re open for read
control_file = open("control", 'w')
control_file.write("0")
control_file.close()
print("initialized control file")
control_file = open("control", "r")

# test control file for text != "0"
while True:
    exit_flag = control_file.read()
    if exit_flag != "0":    # if True take appropriate action
        print("quitting")
        break
    control_file.seek(0)
