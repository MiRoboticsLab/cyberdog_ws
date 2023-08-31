# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM arm64v8/ubuntu:bionic
ENV CMAKE_VER="3.21.2"
ENV GITHUB_URL="github.com"
ENV GITHUB_RAW="raw.githubusercontent.com"

WORKDIR home/builder
# replace mirror (replace one near you)
RUN apt-get update \
&& apt-get install -q -y --no-install-recommends wget \
&& wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/config-deb.tar.gz --no-check-certificate \
&& tar -xzvf config-deb.tar.gz \
&& dpkg -i config-deb/ca-certificates/*.deb \
&& rm -rf /var/lib/apt/lists/*

RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse\n" > /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list
  
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/config-deb.tar.gz && \
  tar -xzvf config-deb.tar.gz && \
  dpkg -i config-deb/tzdata/*.deb && \
  rm -rf *

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive

# install packages
RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/base-deb.tar.gz \
&& tar -xzvf base-deb.tar.gz \
&& cp base-deb/*.deb /var/cache/apt/archives/ \
&& apt install --no-install-recommends -y --allow-downgrades /var/cache/apt/archives/*.deb \
&& rm -rf *

# change pip mirror (replace one near you)
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple pip -U \
&& pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros2-latest.list 

RUN mkdir carpo-ros2-debs \
&& wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/carpo-ros2-debs/carpo-ros2-debs.tgz \
&& tar -xf carpo-ros2-debs.tgz -C carpo-ros2-debs \
&& dpkg -i carpo-ros2-debs/*.deb \
&& rm -rf *

RUN mkdir -p /opt/nvidia/l4t-packages/ \
&& touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall \
&& rm /var/lib/dpkg/info/* -rf 

RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/docker-depend.tar.gz \
&& tar -xzvf docker-depend.tar.gz \
&& dpkg -i docker-depend/dpkg-deb/*.deb \
&& cp docker-depend/apt-deb/*.deb /var/cache/apt/archives/ \
&& apt install --no-install-recommends -y --allow-downgrades /var/cache/apt/archives/*.deb \
&& python3 -m pip install --no-index --find-link ./docker-depend/whls/ -r ./docker-depend/whls/requirement.txt --ignore-installed \
&& cp docker-depend/config-file/libwebrtc.a /usr/local/lib \
&& cp docker-depend/config-file/libgalaxy-fds-sdk-cpp.a /usr/local/lib/ \
&& cp -r docker-depend/config-file/webrtc_headers/ /usr/local/include/ \
&& cp -r docker-depend/config-file/include/* /usr/local/include/ \
&& cp -r docker-depend/config-file/grpc-archive/* /usr/local/lib/ \
&& cp docker-depend/config-file/ldconf/* /etc/ld.so.conf.d \
&& ldconfig \
&& rm -rf * 

RUN rm -f /usr/bin/python \
&& ln -s /usr/bin/python3 /usr/bin/python

# set ros 2 environment configs
RUN echo "ros2_galactic_on(){" >> /root/.bashrc && \
  echo "export ROS_VERSION=2" >> /root/.bashrc && \
  echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc && \
  echo "export ROS_DISTRO=galactic" >> /root/.bashrc && \
  echo "source /opt/ros2/galactic/setup.bash" >> /root/.bashrc && \
  echo "}" >> /root/.bashrc

CMD ["bash"]
