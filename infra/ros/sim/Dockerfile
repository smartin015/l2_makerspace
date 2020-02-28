from l2base:latest

# Gazebo installation
RUN bash -c "source /opt/ros/dashing/setup.bash && apt-get update && apt-get install -y ros-dashing-gazebo-ros ros-dashing-gazebo-ros-pkgs && rm -rf /var/lib/apt/lists/*"

RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

# RUN wget https://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz && tar -xvf CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz && rm CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz
# RUN wget http://download.qt.io/official_releases/qt/5.12/5.12.0/qt-opensource-linux-x64-5.12.0.run
# RUN apt-get update && apt-get install liblua5.1-dev libqt5gui5
