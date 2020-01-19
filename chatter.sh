#!/bin/bash
while true; do
rostopic pub /chatter std_msgs/String "Hi" -1
done
