FROM osrf/ros:noetic-desktop-full AS base

RUN apt update
RUN apt upgrade -y
RUN useradd -m devcontainer

FROM base AS devel

RUN apt install -y wget
RUN wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
RUN sudo dpkg -i packages-microsoft-prod.deb
RUN rm packages-microsoft-prod.deb
RUN apt update

RUN apt install -y g++ git cmake sudo curl dotnet-sdk-6.0 unzip
RUN apt autoremove -y

RUN wget https://github.com/clangd/clangd/releases/download/16.0.2/clangd-linux-16.0.2.zip -O /home/devcontainer/clangd.zip
RUN unzip /home/devcontainer/clangd.zip -d /home/devcontainer/clangd

RUN usermod -aG sudo devcontainer
RUN echo "devcontainer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/devcontainer/.bashrc
RUN echo "source /workspaces/WRoverPlayground/devel/setup.bash" >> /home/devcontainer/.bashrc
RUN echo "export PATH=$PATH:/home/devcontainer/clangd/clangd_16.0.2/bin" >> /home/devcontainer/.profile

USER devcontainer
