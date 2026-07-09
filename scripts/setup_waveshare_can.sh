#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Default serial port
PORT="/dev/ttyUSB0"

# Allow overriding serial port via command line argument
if [ -n "$1" ]; then
    PORT="$1"
fi

echo "============================================="
echo "Waveshare USB-CAN-A ve SocketCAN Kurulumu"
echo "============================================="

# 1. Port varlığını kontrol et
if [ ! -e "$PORT" ]; then
    echo "Hata: '$PORT' seri portu bulunamadı!"
    echo "Bağlı olabilecek USB seri cihazları:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "Hiçbir USB seri cihazı bulunamadı."
    echo "Kullanım: $0 <seri_port_yolu> (Örn: $0 /dev/ttyUSB0)"
    exit 1
fi

echo "Seçilen port: $PORT"

# 2. dialout grubu kontrolü
ACTUAL_USER="${SUDO_USER:-$USER}"
if ! groups "$ACTUAL_USER" | grep -q "\bdialout\b"; then
    echo "UYARI: Kullanıcınız ($ACTUAL_USER) 'dialout' grubunda görünmüyor."
    echo "Porta sudo olmadan erişebilmek için şu komutla kendinizi gruba eklemelisiniz:"
    echo "  sudo usermod -aG dialout $ACTUAL_USER"
    echo "Ardından sistemi yeniden başlatmalı veya oturumu kapatıp açmalısınız."
    echo "============================================="
fi

# 3. vcan modülünün yüklenmesi ve eski arayüzün temizlenmesi
echo "1. vcan çekirdek modülü yükleniyor..."
sudo modprobe vcan

# 4. can0 arayüzünün varlığı ve temizliği
if ip link show can0 >/dev/null 2>&1; then
    echo "Mevcut can0 arayüzü algılandı. Temizleniyor..."
    sudo ip link set down can0 || true
    sudo ip link delete can0 || true
    # Eski köprü betiklerini sonlandır
    sudo pkill -f "waveshare_socketcan_bridge.py" || true
    sleep 1
fi

# 5. Sanal can0 SocketCAN arayüzünün oluşturulması
echo "2. Sanal can0 SocketCAN arayüzü oluşturuluyor..."
sudo ip link add dev can0 type vcan
sudo ip link set up can0

# 6. Waveshare <-> SocketCAN Python Köprüsünün başlatılması
echo "3. Python SocketCAN Köprüsü arka planda başlatılıyor..."
sudo nohup python3 /home/gunes/robotaksi_ws/scripts/waveshare_socketcan_bridge.py "$PORT" can0 > /tmp/waveshare_bridge.log 2>&1 &

# 7. Sonuç doğrulaması
echo "============================================="
echo "Kurulum başarıyla tamamlandı!"
echo "Python köprüsü arka planda çalışıyor. Loglar /tmp/waveshare_bridge.log adresinde."
echo "Arayüz Durumu:"
ip link show can0
echo "============================================="
echo "İlk test için aşağıdaki komutla gelen ham CAN paketlerini izleyebilirsiniz:"
echo "  candump can0"
echo ""
echo "DroneCAN GUI aracını çalıştırmak isterseniz arayüz olarak 'can0'ı seçebilirsiniz."
echo "============================================="
