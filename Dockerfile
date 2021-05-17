# An image for building the EtherCAT master natives.
# Current version: 0.2
FROM ubuntu:xenial

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get --quiet 2 --yes update  \
 && apt-get --quiet 2 --yes install \
    nano \
    git \
    wget \
    curl \
    unzip \
    locales \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    sudo \
    build-essential \
    swig \
    cmake \
    openjdk-8-jdk \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install Gradle
ARG gradleVersion=6.9
RUN curl -sL https://services.gradle.org/distributions/gradle-$gradleVersion-all.zip -o gradle.zip
RUN unzip -q gradle.zip -d gradle
RUN mv gradle/gradle-$gradleVersion/ /opt/.
RUN ln -s /opt/gradle-$gradleVersion/ /opt/gradle
RUN ln -s /opt/gradle/bin/gradle /usr/bin/gradle

RUN apt-add-repository --yes ppa:halodirobotics/ppa  \
 && apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    soem \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Setup a robotlab user as the development user, to avoid using root.
# Allows using sudo with robotlab user without a password.
RUN addgroup robotlab \
 && adduser --home /home/robotlab --gecos "Rosie Robot,1117,1234567,2345678" --ingroup robotlab --disabled-password robotlab \
 && chown -R robotlab /home/robotlab \
 && adduser robotlab sudo \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER robotlab
WORKDIR /home/robotlab

RUN mkdir -p /home/robotlab/.gradle
RUN echo "org.gradle.java.home=/usr/lib/jvm/java-8-openjdk-amd64" > /home/robotlab/.gradle/gradle.properties
ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64

RUN mkdir -p /home/robotlab/dev/ihmc-ethercat-master/build
RUN mkdir -p /home/robotlab/dev/ihmc-ethercat-master/src
RUN mkdir -p /home/robotlab/dev/ihmc-ethercat-master/swig
VOLUME /home/robotlab/dev/ihmc-ethercat-master/build
VOLUME /home/robotlab/dev/ihmc-ethercat-master/src
VOLUME /home/robotlab/dev/ihmc-ethercat-master/swig
WORKDIR /home/robotlab/dev/ihmc-ethercat-master

ENTRYPOINT ["/bin/bash"]
CMD ["build.sh"]
