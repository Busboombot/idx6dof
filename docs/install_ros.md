# Installing andConfiguring ROS

* Install Ubuntu 20.04
* Install [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html)


## Install ROS 2

	sudo apt install software-properties-common
	sudo add-apt-repository universe
	
	sudo apt update && sudo apt -y  install curl gnupg lsb-release
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	
	# Download ros
	sudo mkdir -p /proj/downloads
	sudo chown $USER /proj/downloads
	cd /proj/downloads
	
	wget https://github.com/ros2/ros2/releases/download/release-foxy-20220208/ros2-foxy-20220208-linux-focal-amd64.tar.bz2
	
	sudo mkdir -p /proj/ros2
	sudo chown $USER /proj/ros2
	cd /proj/ros2
	tar xf /proj/downloads/ros2-foxy-20220208-linux-focal-amd64.tar.bz2
	
	sudo apt update
	sudo apt install -y python3-rosdep rti-connext-dds-5.3.1
	sudo rosdep init
	rosdep update
	
	CHOOSE_ROS_DISTRO=foxy # or bouncy
	rosdep install --from-paths /proj/ros2/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
	
	sudo apt install -y libpython3-dev python3-pip
	
Then, add this to ~/.bashrc

	# To set up the DDS provider, or somesuch
	/opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash
	# Setup ROS
	source /proj/ros2/ros2-linux/setup.bash 
	