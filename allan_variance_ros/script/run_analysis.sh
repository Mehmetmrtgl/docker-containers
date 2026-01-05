#!/bin/bash

set -e

if [ "$#" -ne 2 ]; then
    echo "Kullanım: $0 [bag_dosyası_adı] [config_dosyası_adı]"
    echo "Örnek: $0 imu_data.bag imu_config.yaml"
    exit 1
fi

BAG_NAME=$1
CONFIG_NAME=$2

DATA_DIR="/data/ros1bag"
CONFIG_PATH="/root/catkin_ws/src/allan_variance_ros/config/$CONFIG_NAME"
ORIGINAL_BAG="$DATA_DIR/$BAG_NAME"
COOKED_BAG="$DATA_DIR/cooked_$BAG_NAME"

echo "--- 1. ADIM: roscore kontrol ediliyor ---"

if ! rostopic list > /dev/null 2>&1; then
    echo "roscore çalışmıyor. Arka planda başlatılıyor..."
    
    roscore > /dev/null 2>&1 &
    
    echo "roscore'un hazır olması bekleniyor (5 sn)..."
    sleep 5
    
    if ! rostopic list > /dev/null 2>&1; then
        echo "Hata: roscore başlatılamadı!"
        exit 1
    fi
    echo "roscore başarıyla başlatıldı."
else
    echo "roscore zaten çalışıyor."
fi

echo "--- 2. ADIM: Bag dosyası timestamp düzenleme (cookbag) ---"
rosrun allan_variance_ros cookbag.py --input "$ORIGINAL_BAG" --output "$COOKED_BAG"

echo "--- 3. ADIM: Allan Variance hesaplanıyor ---"
rosrun allan_variance_ros allan_variance "$DATA_DIR" "$CONFIG_PATH"

echo "--- 4. ADIM: Analiz ve Görselleştirme ---"

CSV_PATH="$BAG_FOLDER/allan_variance.csv"

if [ -f "$CSV_PATH" ]; then
    echo "CSV dosyası bulundu: $CSV_PATH"
    echo "Görselleştirme başlatılıyor..."
    
    DISPLAY=$DISPLAY rosrun allan_variance_ros analysis.py --data "$CSV_PATH" --config "$CONFIG_PATH"
    
    cp "$CSV_PATH" /data/allan_variance_results.csv
    
    echo "Sonuçlar /data/allan_variance_results.csv olarak kopyalandı."
else
    echo "Hata: $CSV_PATH bulunamadı!"
    echo "Mevcut konumda aranan dosya yok. Lütfen /data/ros1bag içini kontrol edin."
    killall -9 roscore rosmaster > /dev/null 2>&1
    exit 1
fi

echo "--- TEMİZLİK: Süreçler kapatılıyor ---"

killall -9 roscore rosmaster rosout > /dev/null 2>&1

echo "==========================================="
echo "İşlem başarıyla tamamlandı."
echo "Sonuç grafiklerini ve CSV dosyasını /data klasöründe bulabilirsiniz."
echo "==========================================="
