cd /workspace
git clone https://github.com/pjreddie/darknet
cd darknet
git checkout 508381b37f
echo "export DARKNET_HOME=/home/developer/workspace/darknet" >> /home/developer/.bashrc
source /home/developer/.bashrc
cd /workspace
