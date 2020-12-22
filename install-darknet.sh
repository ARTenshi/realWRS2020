cd /workspace
git clone https://github.com/pjreddie/darknet
cd darknet
git checkout 508381b37f
export DARKNET_HOME=/workspace/darknet
echo "export DARKNET_HOME=/workspace/darknet" >> ~/.bashrc
cd /workspace
