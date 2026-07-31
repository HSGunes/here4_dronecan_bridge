#!/usr/bin/env python3
import serial
import socket
import struct
import sys
import select
import threading
import time


def configure_adapter(ser, can_baud=0x01, frame_type=0x02, mode=0x00):
    """Waveshare USB-CAN-A 'settings' (CAN Configuration) komutunu gönderir.

    KRİTİK: Bu komut GÖNDERİLMEZSE adaptör 'silent/listen-only' modda kalabilir
    (en son kayıtlı ayara göre). O modda CAN frame'lerini ALIR (RXD yanar,
    candump görür) ama ACK VERMEZ. can0 bir vcan olduğu için ACK'i sadece bu
    adaptör (donanım) üretebilir; ACK yoksa Here4 çok-parçalı transferin 1.
    frame'ini bus hızında sonsuza dek retransmit eder, transfer hiç tamamlanmaz,
    DroneCAN mesajı decode edilmez ve /here4/* topic'leri hep boş kalır.
    NORMAL mode + doğru bitrate göndermek bunu çözer (sahada kanıtlandı).

    Ref: Waveshare 'Secondary Development Serial Conversion Definition of CAN
    Protocol' — 20 byte CAN Configuration Command.
      byte2 Type  = 0x12  variable-protocol ayarı (bu köprü variable protokol kullanır)
      byte3 baud  = 0x01  1 Mbps (Here4/DroneCAN ve bu robotun CAN bus'ı; throughput ile doğrulandı)
      byte4 frame = 0x02  extended (DroneCAN 29-bit ID)
      byte5-12    = 0     filtre+maske = hepsini kabul et
      byte13 mode = 0x00  NORMAL (ACK üretir) — silent değil!
      byte14      = 0x00  auto-retransmit açık
      byte19      = checksum = sum(byte[2..18]) & 0xFF
    """
    frame = bytearray(20)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = 0x12
    frame[3] = can_baud
    frame[4] = frame_type
    frame[13] = mode
    frame[14] = 0x00
    frame[19] = sum(frame[2:19]) & 0xFF
    ser.write(bytes(frame))
    ser.flush()
    baud_label = {
        0x01: "1Mbps",
        0x02: "800k",
        0x03: "500k",
        0x04: "400k",
        0x05: "250k",
        0x06: "200k",
        0x07: "125k",
        0x08: "100k",
    }.get(can_baud, f"code={can_baud:#x}")
    print(
        f"Adapter yapılandırıldı: {baud_label}, extended, NORMAL(ACK), "
        f"variable-protocol (settings frame: {frame.hex(' ')})",
        flush=True,
    )
    time.sleep(0.1)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <serial_port> <can_interface>")
        print(f"Example: {sys.argv[0]} /dev/ttyUSB2 can0")
        sys.exit(1)

    port = sys.argv[1]
    interface = sys.argv[2]
    # Opsiyonel 3. arg: CAN bitrate kodu (hex). Default 0x01 = 1 Mbps (bu robot).
    # Farklı bus için: 0x03=500k, 0x05=250k, 0x07=125k ...
    can_baud = int(sys.argv[3], 0) if len(sys.argv) >= 4 else 0x01
    # Opsiyonel 4. arg: TX frame'leri arası bekleme (ms). Adaptörün TX tamponu
    # taşmasın diye konmuştu; ama RTCM düzeltmelerinin gecikmesini bu belirliyor:
    # 1 RTCM parçası = 20 CAN frame, yani parça başına 20*tx_sleep ms.
    # 28.07.2026 saha ölçümü (budget=4 sabit, RTK FIXED'e geçiş süresi):
    #   2.0 ms -> ~100 s   1.0 ms -> ~60 s
    # Her ikisinde de CAN seviyesinde (candump) IMU akışı temiz: 0 boşluk,
    # max aralık ~20 ms. DİKKAT: IMU boşluğunu Python ile ölçmeyin — CPU yüklü
    # sistemde ölçüm node'unun kendi gecikmesi sahte boşluk raporlar; bu tuzak
    # 1.0 ms'i bir kez haksız yere eledi. Doğru katman: candump.
    # 0.5 ms de denendi, ek kazanç görülmedi. Çok düşürmek adaptörde frame
    # kaybı yapabilir (crc_hata / dusen sayaçlarından izlenir).
    tx_sleep = (float(sys.argv[4]) / 1000.0) if len(sys.argv) >= 5 else 0.001

    # Setup SocketCAN
    try:
        can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_sock.bind((interface,))
        can_sock.setblocking(False)
    except Exception as e:
        print(
            f"Error binding to SocketCAN interface '{interface}': {e}", file=sys.stderr
        )
        print(
            "Make sure the interface exists and is UP (e.g. 'sudo ip link add dev can0 type vcan && sudo ip link set up can0')",
            file=sys.stderr,
        )
        sys.exit(1)

    # Setup Serial
    try:
        ser = serial.Serial(port, 2000000, timeout=0.01)
    except Exception as e:
        print(f"Error opening serial port '{port}': {e}", file=sys.stderr)
        sys.exit(1)

    # Adaptörü NORMAL moda + doğru bitrate'e al (yoksa silent modda ACK vermez,
    # Here4 saplanır — bkz. configure_adapter docstring).
    try:
        configure_adapter(ser, can_baud=can_baud)
    except Exception as e:
        print(f"Adapter yapılandırma hatası: {e}", file=sys.stderr)

    print(
        f"Bridging {port} <-> {interface} at 2,000,000 bps (Waveshare proprietary protocol)..."
    )

    CAN_FRAME_FMT = "=IB3x8s"
    CAN_EFF_FLAG = 0x80000000
    CAN_RTR_FLAG = 0x40000000

    buffer = bytearray()

    # TX (SocketCAN -> seri) AYRI THREAD'DE.
    # Tek thread'de yapılınca her frame'deki 2 ms'lik anti-taşma beklemesi
    # seri okumayı da durduruyordu: RTCM gibi çok-frame'li bir transfer
    # gönderilirken CH340 giriş tamponu taşıp Here4'ten gelen 100 Hz IMU
    # frame'leri düşüyordu (ölçüm: 686 ms'lik IMU boşluğu, tam RTCM anına
    # denk geliyor). Aynı tıkanma RTCM'i de geciktirip RTK FIXED'i yavaşlatıyordu.
    tx_stop = threading.Event()
    ser_write_lock = threading.Lock()

    def tx_worker():
        while not tx_stop.is_set():
            try:
                r, _, _ = select.select([can_sock], [], [], 0.05)
                if not r:
                    continue
                frame, _ = can_sock.recvfrom(16)
                if not frame:
                    continue
                can_id, dlc, padded_data = struct.unpack(CAN_FRAME_FMT, frame)
                data_bytes = padded_data[:dlc]

                is_extended = bool(can_id & CAN_EFF_FLAG)
                is_remote = bool(can_id & CAN_RTR_FLAG)

                type_byte = 0xC0
                if is_extended:
                    type_byte |= 0x20
                if is_remote:
                    type_byte |= 0x10
                type_byte |= dlc & 0x0F

                if is_extended:
                    id_bytes = (can_id & 0x1FFFFFFF).to_bytes(4, byteorder="little")
                else:
                    id_bytes = (can_id & 0x7FF).to_bytes(2, byteorder="little")

                serial_packet = bytearray([0xAA, type_byte])
                serial_packet.extend(id_bytes)
                serial_packet.extend(data_bytes)
                serial_packet.append(0x55)

                with ser_write_lock:
                    ser.write(serial_packet)
                # Adaptörün TX tamponunu taşırmamak için gerekli; artık yalnız
                # bu thread'i bekletiyor, seri okuma etkilenmiyor.
                if tx_sleep > 0:
                    time.sleep(tx_sleep)
            except BlockingIOError:
                pass
            except Exception as e:
                if not tx_stop.is_set():
                    print(f"TX thread error: {e}", file=sys.stderr)
                break

    tx_thread = threading.Thread(target=tx_worker, daemon=True)
    tx_thread.start()

    try:
        while True:
            # RX: yalnızca seri portu bekle — TX artık ayrı thread'de.
            r, _, _ = select.select([ser], [], [], 0.05)

            # 1. From Serial to SocketCAN
            if ser in r:
                try:
                    # Read available bytes
                    new_data = ser.read(ser.in_waiting or 1)
                    if new_data:
                        buffer.extend(new_data)
                except Exception as e:
                    print(f"Serial read error: {e}", file=sys.stderr)
                    break

                # Process all complete packets in the buffer
                while len(buffer) >= 2:
                    # Find the next 0xAA header
                    try:
                        start_idx = buffer.index(0xAA)
                    except ValueError:
                        buffer.clear()
                        break

                    if start_idx > 0:
                        del buffer[:start_idx]

                    if len(buffer) < 2:
                        break

                    type_byte = buffer[1]
                    is_extended = bool(type_byte & 0x20)
                    is_remote = bool(type_byte & 0x10)
                    dlc = type_byte & 0x0F

                    id_len = 4 if is_extended else 2
                    packet_len = 1 + 1 + id_len + dlc + 1

                    if len(buffer) < packet_len:
                        break  # Wait for more data

                    # Verify end code
                    if buffer[packet_len - 1] != 0x55:
                        # Invalid frame, skip header and search again
                        del buffer[0:1]
                        continue

                    # Parse ID
                    id_bytes = buffer[2 : 2 + id_len]
                    can_id = int.from_bytes(id_bytes, byteorder="little")
                    if is_extended:
                        can_id |= CAN_EFF_FLAG
                    if is_remote:
                        can_id |= CAN_RTR_FLAG

                    # Parse Data
                    data_bytes = buffer[2 + id_len : 2 + id_len + dlc]

                    # Send to SocketCAN
                    try:
                        padded_data = data_bytes.ljust(8, b"\x00")
                        frame = struct.pack(CAN_FRAME_FMT, can_id, dlc, padded_data)
                        can_sock.send(frame)
                    except Exception:
                        # Non-blocking send might occasionally fail if buffer is full
                        pass

                    # Consume packet
                    del buffer[:packet_len]

    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        tx_stop.set()
        tx_thread.join(timeout=1.0)
        ser.close()
        can_sock.close()


if __name__ == "__main__":
    main()
