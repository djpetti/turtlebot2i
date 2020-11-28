# Deploys the currently-built code to the turtlebot.

set -e

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 ROBOT_IP"
  exit 1
fi

robot_ip=$1
rsync -rvz . ${robot_ip}:turtlebot2i