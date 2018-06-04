FROM arm32v7/ros:kinetic-perception-xenial

RUN apt-get clean && apt-get update && apt-get upgrade -y

RUN apt-get install -yq --no-install-recommends --fix-missing \
    sudo \
    etckeeper emacs vim byobu zsh git git-extras htop atop nethogs iftop apt-file ntpdate \
    build-essential libblas-dev liblapack-dev libatlas-base-dev gfortran libyaml-cpp-dev

RUN apt-get install -yq --no-install-recommends --fix-missing \
    python-dev ipython python-sklearn python-smbus python-termcolor python-frozendict \
    python-tables i2c-tools libxslt-dev libxml2-dev python-lxml python-bs4 

RUN apt-get install -yq --no-install-recommends --fix-missing \
    ros-kinetic-ros-tutorials ros-kinetic-common-tutorials ros-kinetic-robot=1.3.2-0*

RUN rm -rf /var/lib/apt/lists

RUN easy_install pip

RUN pip install --upgrade --user \
        PyContracts==1.7.15 \
        DecentLogs==1.1.2\
        QuickApp==1.3.8 \
        conftools==1.9.1 \
        comptests==1.4.10 \
        procgraph==1.10.6 \
        pymongo==3.5.1 \
        ruamel.yaml==0.15.34

RUN git clone https://github.com/duckietown/software /home/

RUN source /opt/ros/kinetic/setup.bash && catkin_make -C /home/software/catkin_ws/

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

CMD ["bash"]
