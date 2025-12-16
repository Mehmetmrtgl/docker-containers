FROM osrf/ros:humble-desktop

# Dil ve Klavye Ayarları
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# 1. Temel Araçların Kurulumu
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    nano \
    curl \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# 2. Kullanıcı Ayarları (Root Sorununu Çözmek İçin)
# Varsayılan olarak UID 1000 (Genelde Linux'ta ilk kullanıcı) atanır.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Kullanıcıyı oluştur ve sudo yetkisi ver (şifresiz)
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 3. Ortam Değişkenleri
# Kullanıcı her terminal açtığında ROS 2 otomatik source edilsin
USER $USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

# Çalışma Alanı
WORKDIR /home/$USERNAME/ros2_ws

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
