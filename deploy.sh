echo "Deploying..."
rsync -a . ubuntu@10.7.51.76:~/Barn2ROS.current
echo "Deployed Successfully"