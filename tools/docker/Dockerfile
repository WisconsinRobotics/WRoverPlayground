FROM osrf/ros:noetic-desktop-full AS base

RUN apt update
RUN apt upgrade -y
RUN useradd -m devcontainer

FROM base AS devel

RUN apt install -y wget bash-completion

# Add Microsoft repo for dotnet
RUN wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
RUN sudo dpkg -i packages-microsoft-prod.deb
RUN rm packages-microsoft-prod.deb
RUN apt update

# Install build dependencies
RUN apt install -y g++ git cmake sudo curl dotnet-sdk-6.0 unzip
# Install python dependencies
RUN apt install -y python-is-python3 python3-pip python3-venv python3-tk

RUN apt autoremove -y

# Install clangd for code completion
RUN wget https://github.com/clangd/clangd/releases/download/16.0.2/clangd-linux-16.0.2.zip -O /home/devcontainer/clangd.zip
RUN unzip /home/devcontainer/clangd.zip -d /home/devcontainer/clangd

RUN usermod -aG sudo devcontainer
RUN echo "devcontainer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers

# Configure bash to set up ROS environment and project
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/devcontainer/.bashrc
RUN echo "source /workspaces/WRoverPlayground/devel/setup.bash" >> /home/devcontainer/.bashrc
RUN echo "if [ -d /workspaces/WRoverPlayground/venv ]; then source /workspaces/WRoverPlayground/venv/bin/activate; fi" >> /home/devcontainer/.bashrc
RUN echo "export PATH=$PATH:/home/devcontainer/clangd/clangd_16.0.2/bin" >> /home/devcontainer/.profile

USER devcontainer

ENV SHELL /bin/bash
