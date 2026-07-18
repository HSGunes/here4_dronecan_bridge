#!/usr/bin/env python3
import serial
import socket
import struct
import sys
import select
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

    try:
        while True:
            # Use select to wait for data on serial or socket (with a short timeout)
            r, _, _ = select.select([ser, can_sock], [], [], 0.05)

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

            # 2. From SocketCAN to Serial
            if can_sock in r:
                try:
                    frame, _ = can_sock.recvfrom(16)
                    if frame:
                        can_id, dlc, padded_data = struct.unpack(CAN_FRAME_FMT, frame)
                        data_bytes = padded_data[:dlc]

                        is_extended = bool(can_id & CAN_EFF_FLAG)
                        is_remote = bool(can_id & CAN_RTR_FLAG)

                        # Construct Type byte
                        type_byte = 0xC0
                        if is_extended:
                            type_byte |= 0x20
                        if is_remote:
                            type_byte |= 0x10
                        type_byte |= dlc & 0x0F

                        # Pack ID
                        if is_extended:
                            raw_id = can_id & 0x1FFFFFFF
                            id_bytes = raw_id.to_bytes(4, byteorder="little")
                        else:
                            raw_id = can_id & 0x7FF
                            id_bytes = raw_id.to_bytes(2, byteorder="little")

                        # Construct serial packet
                        serial_packet = bytearray([0xAA, type_byte])
                        serial_packet.extend(id_bytes)
                        serial_packet.extend(data_bytes)
                        serial_packet.append(0x55)

                        ser.write(serial_packet)
                        import time

                        time.sleep(
                            0.002
                        )  # 2ms delay to prevent buffer overflow on multi-frame TX
                except BlockingIOError:
                    pass
                except Exception as e:
                    print(f"SocketCAN read/write error: {e}", file=sys.stderr)
                    break
    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        ser.close()
        can_sock.close()


if __name__ == "__main__":
    main()
