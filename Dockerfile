FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

RUN mkdir /home/software
COPY . /home/software/

RUN /bin/bash -c "source /home/software/environment.sh && catkin_make -C /home/software/catkin_ws/"

RUN echo $'\n\
    source /home/software/environment.sh \n\
    export DUCKIEFLEET_ROOT=/home/duckiefleet \n\
    cd /home/software \n\
    ' >> ~/.bashrc

RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet

RUN [ "cross-build-end" ]

CMD ["bash"]
