FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

ENV ROS_MASTER_URI http://localhost:11311/

RUN mkdir /home/software
COPY . /home/software/

RUN echo '\n\
source /home/software/environment.sh \n\
source /home/software/set_ros_master.sh \n\
export DUCKIEFLEET_ROOT=/home/duckiefleet \n\
cd /home/software' >> ~/.bashrc

RUN /bin/bash -c "cd /home/software/ && source environment.sh && catkin_make -C catkin_ws/"

# RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet
# RUN echo 'owner: docker\n\
# hostname: localhost\n\
# username: root\n\
# description: Docker configuration.\n\
# log:' >> /home/duckiefleet/robots/docker.robot.yaml
# RUN /bin/bash -c "cd /home/software/ && source environment.sh && make build-machines"

RUN echo '<launch> <arg name="env_script_path" default="~/duckietown/environment.sh"/> <machine name="docker" address="localhost" user="root" env-loader="$(arg env_script_path)"/></launch>' > /home/software/catkin_ws/src/00-infrastructure/duckietown/machines

# RUN ssh-keygen -q -t rsa -N '' -f /root/.ssh/id_rsa && ssh-copy-id -i ~/.ssh/id_rsa.pub localhost

RUN [ "cross-build-end" ]
