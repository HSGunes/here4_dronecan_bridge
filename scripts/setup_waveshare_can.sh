#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Port Auto-Detection
if [ -n "$1" ]; then
    PORT="$1"
else
    echo "Port belirtilmedi. Otomatik olarak CH340 (Waveshare) cihazı aranıyor..."
    ch340_ports=()
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$dev" ] && udevadm info -q property -n "$dev" 2>/dev/null | grep -q "ID_VENDOR_ID=1a86"; then
            ch340_ports+=("$dev")
        fi
    done
    
    if [ ${#ch340_ports[@]} -eq 1 ]; then
        PORT="${ch340_ports[0]}"
        echo "-> Otonom Tespit Başarılı: $PORT (CH340 Çipi Doğrulandı)"
    elif [ ${#ch340_ports[@]} -gt 1 ]; then
        # Arduino ve Waveshare ikisi de CH340 (1a86:7523) — kimlikçe ayırt
        # edilemez ve takışta port numarası (ttyUSB0/1) yer değiştirir. O yüzden
        # İÇERİKTEN ayırt et: hangi port 2M'de 0xAA çerçeveli CAN trafiği yayıyorsa
        # Waveshare odur (Arduino ASCII telemetri yayar). Port numarası önemsizleşir.
        echo "============================================="
        echo "Birden fazla CH340 bulundu: ${ch340_ports[*]}"
        echo "0xAA CAN trafiği yayan Waveshare portu içerikten aranıyor..."
        PORT="$(python3 - "${ch340_ports[@]}" <<'PYEOF'
import serial, sys, time
best_port, best_cnt = "", 0
for p in sys.argv[1:]:
    try:
        s = serial.Serial(p, 2000000, timeout=0.3)
        buf = bytearray(); t = time.time()
        while time.time() - t < 0.8:
            buf.extend(s.read(512))
        s.close()
        c = buf.count(0xAA)
    except Exception:
        c = 0
    print(f"   {p}: 0xAA={c}", file=sys.stderr)
    if c > best_cnt:
        best_cnt, best_port = c, p
if best_cnt > 5:
    print(best_port)
PYEOF
)"
        if [ -n "$PORT" ]; then
            echo "-> Waveshare otomatik seçildi (0xAA trafiği): $PORT"
        else
            echo "Otomatik seçilemedi (0xAA yayan port yok)."
            echo -n "Lütfen Waveshare olan portu yazın (Örn: /dev/ttyUSB2): "
            read -r PORT
        fi
    else
        PORT="/dev/ttyUSB0"
        echo "UYARI: CH340 donanım kimliği bulunamadı, varsayılan $PORT deneniyor..."
    fi
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
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
sudo nohup python3 "$SCRIPT_DIR/waveshare_socketcan_bridge.py" "$PORT" can0 > /tmp/waveshare_bridge.log 2>&1 &

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
