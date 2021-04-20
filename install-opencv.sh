cd /workspace/src/wrs2020
sudo apt-get install unzip
unzip toinstall/opencv-3.4.3.zip -d /workspace
unzip toinstall/opencv_contrib-3.4.3.zip -d /workspace
echo -e "${FRM}${WHITE}${BGBLUE} Installing OpenCV 3.4.3 ${NC}"
cd /workspace/opencv-3.4.3
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPEPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D CMAKE_BUILD_TYPE=RELEASE FORCE_VTK=ON -D WITH_CUBLAS=ON -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.3/modules ..
make -j4
sudo make install
sudo touch /etc/ld.so.conf.d/opencv.conf
sudo /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
sudo ldconfig
echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.4.3 has been installed ${NC}"
cd /workspace
