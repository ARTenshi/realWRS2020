sudo sh -c 'echo "deb https://packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
sudo sh -c 'echo "deb https://packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo sh -c 'echo "deb https://apache.bintray.com/couchdb-deb `lsb_release -cs` main" > /etc/apt/sources.list.d/couchdb.list'
wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
wget https://couchdb.apache.org/repo/bintray-pubkey.asc -O - | sudo apt-key add -
sudo sh -c 'mkdir -p /etc/apt/auth.conf.d'
sudo sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
sudo apt-get update
sudo apt-get install -y ros-melodic-hsrb-interface-py
sudo apt-get install -y libpcl-dev
sudo apt-get install -y libpcl-conversions-dev
sudo apt-get install -y ros-melodic-pcl-conversions
sudo apt-get install -y ros-melodic-pcl-ros
sudo apt-get install -y ros-melodic-image-transport-plugins
sudo apt-get install -y ros-melodic-vision-opencv
sudo apt-get install -y ros-melodic-map-server
sudo apt-get install -y ros-melodic-tf2-bullet
sudo apt-get install -y ros-melodic-sound-play
sudo apt-get install -y ros-melodic-amcl
sudo apt-get install -y ros-melodic-gmapping
sudo apt-get install -y python-numpy
sudo apt-get install -y zbar-tools
sudo apt-get install -y build-essential libzbar-dev
sudo apt-get install -y ros-melodic-smach ros-melodic-smach-viewer

cd /workspace/catkin_ws/src/wrs2020
rm -r /workspace/catkin_ws/src/wrs2020/autolab_core
git clone https://github.com/BerkeleyAutomation/autolab_core.git /workspace/catkin_ws/src/wrs2020/autolab_core
cd /workspace/catkin_ws/src/wrs2020/autolab_core
sudo python /workspace/catkin_ws/src/wrs2020/autolab_core/setup.py install

pip install autolab_core
pip install tensorflow==1.2.0 --ignore-installed
pip install reportlab
pip install numpy
pip install zbar

cd /workspace
wget http://dlib.net/files/dlib-19.6.tar.bz2
tar xvf dlib-19.6.tar.bz2
cd dlib-19.6/
mkdir build
cd build
cmake ..
cmake --build . --config Release
sudo make install
sudo ldconfig
cd /workspace
