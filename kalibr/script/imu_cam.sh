#!/bin/bash
set -e

if [ "$#" -ne 1 ]; then
    echo "Kullanım: ./imu_cam.sh <bag_dosyası_tam_yolu>"
    echo "Örnek: ./imu_cam.sh /root/catkin_ws/data/dynamic.bag"
    exit 1
fi

BAG_PATH=$1
# Bag dosyasının bulunduğu klasörü alıyoruz
BAG_DIR=$(dirname "$BAG_PATH")
BAG_FILE=$(basename "$BAG_PATH")
BAG_NAME="${BAG_FILE%.*}"

BASE_DIR="/root/catkin_ws"
DATA_DIR="$BASE_DIR/data"
TARGET_YAML="$BASE_DIR/april_6x6_80x80cm.yaml"

cd "$BASE_DIR"

# En son oluşturulan calibration_X klasörünü bul
LAST_CALIB=$(ls -d $DATA_DIR/calibration_* 2>/dev/null | sort -V | tail -n 1)

if [ -z "$LAST_CALIB" ]; then
    echo "Hata: Hiçbir kalibrasyon klasörü bulunamadı! Önce kamera kalibrasyonunu (cam.sh) yapmalısın."
    exit 1
fi

DYNAMIC_DIR="$LAST_CALIB/dynamic"

if [ -d "$DYNAMIC_DIR" ]; then
    echo "HATA: $DYNAMIC_DIR zaten mevcut! İşlem durduruldu."
    exit 1
fi

mkdir -p "$DYNAMIC_DIR"
echo "Hedef Klasör: $DYNAMIC_DIR"

# Kalibrasyon komutu
rosrun kalibr kalibr_calibrate_imu_camera \
  --imu-models scale-misalignment \
  --reprojection-sigma 1.0 \
  --target "$TARGET_YAML" \
  --imu "$DATA_DIR/static_imu/imu.yaml" \
  --cams "$LAST_CALIB/static/static-camchain.yaml" \
  --bag "$BAG_PATH" \
  --show-extraction

# DÜZELTME: Dosyaları BAG_DIR içinden alıp taşıyoruz
mv "$BAG_DIR/${BAG_NAME}-camchain-imucam.yaml" "$DYNAMIC_DIR/dynamic-camchain-imucam.yaml"
mv "$BAG_DIR/${BAG_NAME}-imu.yaml" "$DYNAMIC_DIR/dynamic-imu.yaml"
mv "$BAG_DIR/${BAG_NAME}-report-imucam.pdf" "$DYNAMIC_DIR/dynamic-report-imucam.pdf"
mv "$BAG_DIR/${BAG_NAME}-results-imucam.txt" "$DYNAMIC_DIR/dynamic-results-imucam.txt"

echo "------------------------------------------"
echo "İşlem Tamamlandı! Dosyalar şuraya taşındı: $DYNAMIC_DIR"
echo "------------------------------------------"
