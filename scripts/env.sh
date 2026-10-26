# Source this file in Bash: source scripts/env.sh
source /home/pi/miniconda3/etc/profile.d/conda.sh
conda activate ros2_env
source "$CONDA_PREFIX/local_setup.bash"

export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42
hash -r
