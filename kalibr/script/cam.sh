#!/bin/bash
set -e

if [ "$#" -ne 1 ]; then
    echo "Kullanım: ./cam.sh <bag_dosyası_adı_veya_yolu>"
    echo "Örnek: ./cam.sh data/my_record.bag"
    exit 1
fi

BAG_PATH=$1
BAG_FILE=$(basename "$BAG_PATH")
BAG_NAME="${BAG_FILE%.*}"


BASE_DIR="/root/catkin_ws"
DATA_DIR="$BASE_DIR/data"
TARGET_YAML="$BASE_DIR/april_6x6_80x80cm.yaml"

cd "$BASE_DIR"

COUNTER=1
while [ -d "$DATA_DIR/calibration_$COUNTER" ]; do
    let COUNTER++
done

CALIB_DIR="$DATA_DIR/calibration_$COUNTER"
STATIC_DIR="$CALIB_DIR/static"

mkdir -p "$STATIC_DIR"

echo "İşlem başlatılıyor: $BAG_NAME"
echo "Hedef klasör: $STATIC_DIR"


rosrun kalibr kalibr_calibrate_cameras \
  --target "$TARGET_YAML" \
  --models pinhole-equi pinhole-equi \
  --topics /zed/zed_node/left/gray/raw/image /zed/zed_node/right/gray/raw/image \
  --bag "$BAG_PATH" \
  --bag-freq 10.0 \
  --show-extraction


mv "${BAG_NAME}-camchain.yaml" "$STATIC_DIR/static-camchain.yaml"
mv "${BAG_NAME}-report-cam.pdf" "$STATIC_DIR/static-report-cam.pdf"
mv "${BAG_NAME}-results-cam.txt" "$STATIC_DIR/static-results-cam.txt"

echo "------------------------------------------"
echo "Bitti! Dosyalar şurada: $STATIC_DIR"
echo "------------------------------------------"
