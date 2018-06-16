FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

RUN mkdir ~/software
COPY . ~/software/

RUN /bin/bash -c "cd ~/software/ && source environment.sh && catkin_make -C catkin_ws/"

RUN echo $'\n\
    source ~/software/environment.sh \n\
    export DUCKIEFLEET_ROOT=~/duckiefleet \n\
    cd ~/software \n\
    ' >> ~/.bashrc

RUN git clone https://github.com/duckietown/duckiefleet ~/duckiefleet

RUN /bin/bash -c "cd ~/software/ && source environment.sh && make build-machines"

RUN [ "cross-build-end" ]
