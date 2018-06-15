FROM duckietown/duckieos

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

RUN mkdir /home/software
COPY . /home/software/

RUN /bin/bash -c "cd /home/software/ && source environment.sh && catkin_make -C catkin_ws/"

RUN echo $'\n\
    source /home/software/environment.sh \n\
    export DUCKIEFLEET_ROOT=/home/duckiefleet \n\
    cd /home/software \n\
    ' >> ~/.bashrc

RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet

RUN /bin/bash -c "cd /home/software/environment.sh && source environment.sh && make build-machines"

RUN [ "cross-build-end" ]
