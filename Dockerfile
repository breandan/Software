FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

RUN mkdir /home/software
COPY . /home/software/

RUN echo '\n\
source /home/software/environment.sh \n\
source /home/software/set_ros_master.sh \n\
export DUCKIEFLEET_ROOT=/home/duckiefleet \n\
cd /home/software' >> ~/.bashrc

RUN /bin/bash -c "cd /home/software/ && source environment.sh && catkin_make -C catkin_ws/"

RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet

RUN echo 'owner: docker\n\
hostname: localhost\n\
username: root\n\
description: Docker configuration.\n\
log:' >> /home/duckiefleet/robots/docker.robot.yaml

ENV DUCKIEFLEET_ROOT=/home/duckiefleet
RUN /bin/bash -c "cd /home/software/ && source environment.sh && make build-machines"

RUN ssh-keygen -q -t rsa -N '' && ssh-copy-id -i ~/.ssh/id_rsa.pub localhost

RUN [ "cross-build-end" ]
