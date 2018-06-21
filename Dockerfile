FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

RUN mkdir /home/software
COPY . /home/software/

RUN echo '\n\
source /home/software/environment.sh \n\
export DUCKIEFLEET_ROOT=/home/duckiefleet \n\
export ROS_MASTER_URI=http://localhost:11311/ \n\
cd /home/software' >> ~/.bashrc

RUN /bin/bash -c "cd /home/software/ && source environment.sh && catkin_make -C catkin_ws/"

RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet

RUN echo '<launch> <arg name="env_script_path" default="~/duckietown/environment.sh"/> <machine name="docker" address="localhost" user="root" env-loader="$(arg env_script_path)"/></launch>' > /home/software/catkin_ws/src/00-infrastructure/duckietown/machines

RUN [ "cross-build-end" ]

CMD [ "/bin/bash" ]

