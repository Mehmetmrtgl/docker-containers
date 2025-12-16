# Temel imaj: ROS Noetic (Ubuntu 20.04)
FROM ros:noetic-ros-base

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# 1. Derleme için gerekli temel kütüphaneleri yükle
# Python'u kaynaktan derlemek için bu kütüphaneler şarttır.
RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    libsqlite3-dev \
    libbz2-dev \
    liblzma-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Python 3.11'i İndir ve Derle
WORKDIR /tmp
RUN wget https://www.python.org/ftp/python/3.11.9/Python-3.11.9.tgz \
    && tar -xf Python-3.11.9.tgz \
    && cd Python-3.11.9 \
    && ./configure --enable-optimizations \
    && make -j$(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.11.9 Python-3.11.9.tgz
# Not: 'make altinstall' komutu sistemin kendi Python'unu (ros için gerekli olanı) bozmaz.
# Yeni python'a 'python3.11' komutuyla erişeceğiz.

# 3. Pip Kurulumu ve Rosbags
# Python 3.11 ile pip'i güncelleyip rosbags kuruyoruz.
RUN python3.11 -m pip install --upgrade pip \
    && python3.11 -m pip install rosbags

# 4. Çalışma Alanı Ayarları
WORKDIR /ros_ws

# 5. Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
