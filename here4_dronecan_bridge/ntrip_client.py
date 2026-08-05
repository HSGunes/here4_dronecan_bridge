#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# NTRIP istemcisi ve RTCM3 çerçeve ayrıştırıcı.
#
# Bu modül BİLEREK ROS'suzdur (sadece stdlib): hem here4_bridge_node içinden
# hem de scripts/ntrip_tusaga_test.py tarafından ROS ortamı olmadan
# kullanılabilsin diye. Mantığı kopyalama — buradan import et.

import base64
import socket
import threading
import time
from collections import Counter

RTCM3_PREAMBLE = 0xD3
_CRC24Q_POLY = 0x1864CFB

# TUSAGA-Aktif (TKGM) varsayılanları. Kaynak: caster sourcetable + TKGM SSS.
DEFAULT_HOST = "212.156.70.42"
DEFAULT_PORT = 2101
# VRSRTCM34: GPS+GLO+GAL+BDS+QZS, RTCM 3.4 — NEO-F9P'nin çok-takımyıldız
# yeteneğini kullanan tek yayın. Yedek: VRSRTCM31 (sadece GPS+GLONASS).
DEFAULT_MOUNTPOINT = "VRSRTCM34"


def crc24q(data: bytes) -> int:
    """RTCM3 çerçevesinin sonundaki 24-bit Qualcomm CRC'si (poly 0x1864CFB)."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= _CRC24Q_POLY
    return crc & 0xFFFFFF


def nmea_checksum(sentence_body: str) -> str:
    """'$' ile '*' arasındaki gövdenin XOR checksum'ı."""
    checksum = 0
    for char in sentence_body:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def build_gga(
    lat_deg: float, lon_deg: float, alt_m: float, geoid_sep_m: float = 0.0
) -> bytes:
    """Caster'a gönderilecek NMEA GGA cümlesi.

    TUSAGA-Aktif'in tüm mountpoint'lerinde sourcetable nmea=1 diyor: VRS sanal
    bazı bu konuma üretilir. Konum kabaca doğru olmalı; metre hassasiyeti
    gerekmez ama yanlış bölge gönderirsen düzeltme işe yaramaz.
    """
    now = time.gmtime()
    hhmmss = time.strftime("%H%M%S", now) + ".00"

    lat_hemi = "N" if lat_deg >= 0 else "S"
    lon_hemi = "E" if lon_deg >= 0 else "W"
    lat_abs, lon_abs = abs(lat_deg), abs(lon_deg)

    # NMEA ddmm.mmmm / dddmm.mmmm formatı
    lat_str = f"{int(lat_abs):02d}{(lat_abs - int(lat_abs)) * 60:09.6f}"
    lon_str = f"{int(lon_abs):03d}{(lon_abs - int(lon_abs)) * 60:09.6f}"

    # Alan 9 = ortometrik (MSL) yükseklik, alan 11 = jeoit ayrımı N.
    # Caster elipsoit yüksekliğini MSL + N olarak hesaplar; N'i 0 göndermek
    # konumumuzu düşey olarak N kadar yanlış bildirmek demektir (Türkiye'de
    # onlarca metre). Fix2 hem height_msl_mm hem height_ellipsoid_mm verdiği
    # için N'i tam olarak hesaplayabiliyoruz: N = elipsoit - MSL.
    # quality=1, sats=10, hdop=1.0 sabit — caster bunları kullanmıyor.
    body = (
        f"GPGGA,{hhmmss},{lat_str},{lat_hemi},{lon_str},{lon_hemi},"
        f"1,10,1.0,{alt_m:.1f},M,{geoid_sep_m:.1f},M,,"
    )
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


class ChunkedDecoder:
    """HTTP chunked transfer-encoding çözücü (RFC 7230 §4.1).

    TUSAGA-Aktif'in "Trimble Ntrip Caster 5.2"si yanıtı
    `Transfer-Encoding: chunked` ile yayınlıyor. Çözülmezse chunk uzunluk
    başlıkları ("1F\\r\\n", "400\\r\\n" gibi ASCII diziler) RTCM akışının
    ORTASINA karışır: hem içine düştüğü çerçeve bozulur hem ayrıştırıcı
    senkronu kaybeder.

    27.07.2026 ham akış ölçümü (23251 bayt): baytların %4.12'si çerçeve
    dışında kalıyor, 127 geçerli çerçeveye karşılık 7 sahte CRC hatası.
    """

    _SIZE, _DATA, _DONE = 0, 1, 2

    def __init__(self):
        self._buffer = bytearray()
        self._state = self._SIZE
        self._remaining = 0
        self.finished = False

    def feed(self, chunk: bytes) -> bytes:
        """Ham socket baytlarını alır, çözülmüş gövde baytlarını döndürür."""
        self._buffer.extend(chunk)
        out = bytearray()

        while True:
            if self._state == self._SIZE:
                index = self._buffer.find(b"\r\n")
                if index < 0:
                    break
                line = bytes(self._buffer[:index])
                del self._buffer[: index + 2]
                if not line:
                    # Bir önceki chunk verisinin ardındaki CRLF
                    continue
                # "1F" ya da "1F;ext=..." biçiminde olabilir
                size_text = line.split(b";", 1)[0].strip()
                try:
                    self._remaining = int(size_text, 16)
                except ValueError:
                    raise ConnectionError(f"Bozuk chunk uzunluk başlığı: {size_text!r}")
                if self._remaining == 0:
                    self.finished = True
                    self._state = self._DONE
                    break
                self._state = self._DATA

            elif self._state == self._DATA:
                take = min(self._remaining, len(self._buffer))
                if take == 0:
                    break
                out.extend(self._buffer[:take])
                del self._buffer[:take]
                self._remaining -= take
                if self._remaining == 0:
                    self._state = self._SIZE

            else:
                break

        return bytes(out)


class Rtcm3Parser:
    """TCP akışından tam ve CRC'si doğrulanmış RTCM3 çerçeveleri ayıklar.

    Çerçeve: 0xD3 | 6 bit rezerv + 10 bit uzunluk | payload | 3 bayt CRC24Q
    Mesaj tipi payload'ın ilk 12 bitidir.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.types = Counter()

    def feed(self, chunk: bytes):
        """Yeni baytları ekler, çözülen her çerçeveyi tek tek verir (generator)."""
        self._buffer.extend(chunk)
        index = 0
        while True:
            start = self._buffer.find(bytes([RTCM3_PREAMBLE]), index)
            if start < 0:
                self._buffer = bytearray()
                return
            if len(self._buffer) - start < 3:
                self._buffer = self._buffer[start:]
                return

            payload_len = ((self._buffer[start + 1] & 0x03) << 8) | self._buffer[
                start + 2
            ]
            frame_len = 3 + payload_len + 3
            if len(self._buffer) - start < frame_len:
                self._buffer = self._buffer[start:]
                return

            frame = bytes(self._buffer[start : start + frame_len])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                # CRC tutmadı: burası çerçeve başlangıcı değilmiş, bir sonrakini ara
                self.crc_errors += 1
                index = start + 1
                continue

            msg_type = (frame[3] << 4) | (frame[4] >> 4)
            self.types[msg_type] += 1
            self.frames += 1
            index = start + frame_len
            yield frame


def open_ntrip(host, port, mountpoint, user, password, timeout_s=15.0):
    """Caster'a bağlanır, HTTP yanıtını doğrular, akıtmaya hazır socket döndürür.

    Dönen: (socket, durum_satiri, chunked_mi). `chunked_mi` True ise ham
    baytlar ChunkedDecoder'dan geçirilmeli — TUSAGA-Aktif böyle yayınlıyor.
    Hata durumunda socket kapatılır ve istisna fırlatılır.
    """
    sock = socket.create_connection((host, port), timeout=timeout_s)
    sock.settimeout(timeout_s)

    request = (
        f"GET /{mountpoint} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP here4_dronecan_bridge/0.1\r\n"
    )
    if user:
        token = base64.b64encode(f"{user}:{password}".encode()).decode()
        request += f"Authorization: Basic {token}\r\n"
    request += "\r\n"

    try:
        sock.sendall(request.encode())

        header = b""
        while True:
            chunk = sock.recv(1)
            if not chunk:
                raise ConnectionError("Caster başlık göndermeden bağlantıyı kapattı")
            header += chunk
            if header.endswith(b"\r\n\r\n"):
                break
            # NTRIP 1.0 casterları boş satır göndermeden gövdeye geçebiliyor
            if header == b"ICY 200 OK\r\n":
                break
            if len(header) > 8192:
                raise ConnectionError("Başlık 8 KB'ı aştı, yanıt beklenmedik")

        header_text = header.decode("latin-1", errors="replace")
        first_line = header_text.split("\r\n", 1)[0]

        if "200" not in first_line:
            if "401" in first_line:
                raise PermissionError(
                    "401 Unauthorized — kullanıcı adı/şifre yanlış veya "
                    "abonelik bu mountpoint'i kapsamıyor"
                )
            raise ConnectionError(f"Beklenmeyen yanıt: {first_line}")

        if "SOURCETABLE" in header_text.upper():
            raise ConnectionError(
                f"Caster mountpoint yerine sourcetable döndü — "
                f"'{mountpoint}' adı yanlış olabilir"
            )
        # Trimble Ntrip Caster 5.2 "Transfer-Encoding: chunked" kullanıyor.
        is_chunked = "transfer-encoding: chunked" in header_text.lower()
    except Exception:
        sock.close()
        raise

    return sock, first_line, is_chunked


def fetch_sourcetable(host=DEFAULT_HOST, port=DEFAULT_PORT, timeout_s=15.0) -> str:
    """Caster'ın public yayın listesini çeker (kimlik bilgisi göndermez)."""
    sock = socket.create_connection((host, port), timeout=timeout_s)
    sock.settimeout(timeout_s)
    try:
        sock.sendall(
            f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP here4_dronecan_bridge/0.1\r\n\r\n".encode()
        )
        data = b""
        while b"ENDSOURCETABLE" not in data:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
    finally:
        sock.close()
    return data.decode("latin-1", errors="replace")


class StallReconnect(ConnectionError):
    """Akış durdu diye bilerek koparıldı — geri çekilme (backoff) uygulanmaz.

    05.08.2026 ölçümü bunu haklı çıkarıyor: 24 s'lik bir durak boyunca AYNI
    caster'a (212.156.70.42:2101, kimliksiz sourcetable) 5 yoklamanın 5'i de
    ortalama 0.19 s'de başarılı oldu. Yani caster ayakta ve yeni bağlantıya
    anında veri veriyor; susan yalnızca bizim akan oturumumuz. Beklemek yerine
    koparıp yeniden bağlanmak ölü süreyi 15-27 s'den ~1 s'ye indirir.
    """


class NtripClient:
    """Arka planda çalışan, kopunca yeniden bağlanan NTRIP istemcisi.

    on_rtcm(frame_bytes)  : CRC'si doğrulanmış her RTCM3 çerçevesi için çağrılır.
                            DİKKAT: NTRIP thread'inden çağrılır — callback
                            thread-safe olmalı (kuyruğa yaz, iş yapma).
    get_position()        : (lat, lon, alt) ya da None. None dönerse GGA
                            gönderilmez; VRS akıtmaya başlamaz.
    """

    def __init__(
        self,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        mountpoint=DEFAULT_MOUNTPOINT,
        user="",
        password="",
        on_rtcm=None,
        get_position=None,
        gga_period_s=5.0,
        logger=None,
        reconnect_max_s=30.0,
        stall_reconnect_s=8.0,
    ):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self._user = user
        self._password = password
        self._on_rtcm = on_rtcm
        self._get_position = get_position
        self._gga_period_s = gga_period_s
        self._log = logger
        self._reconnect_max_s = reconnect_max_s

        self._thread = None
        self._running = False

        # Telemetri (dışarıdan salt-okuma)
        self.connected = False
        self.total_bytes = 0
        self.total_frames = 0
        self.crc_errors = 0
        self.reconnects = 0
        self.last_frame_monotonic = 0.0
        self.types = Counter()

        # Durak (stall) telemetrisi. 05.08.2026 sahada olculdu: TCP ayakta ve
        # ag saglikli (ping RTT durak icinde 53 ms / disinda 54 ms, sifir kayip)
        # oldugu halde caster 5-21 s boyunca hic bayt gondermiyor; olu zaman
        # %13-18. Her durak RTK FIXED kilidini dusuruyor, bu yuzden gorunur
        # olmasi sart.
        self.stall_threshold_s = 3.0
        # Bu kadar bayt gelmezse oturumu koparip yeniden baglan (0 = kapali).
        self.stall_reconnect_s = stall_reconnect_s
        self.stall_reconnects = 0
        self.stalls = 0
        self.unexpected_errors = 0
        self.callback_errors = 0
        # B56: durak koparmayla bitince gercek olu sure ancak veri geri
        # gelince bilinir; baslangici _stream cagrilari arasinda tasiyoruz.
        self._stall_since = None
        self.stalled_seconds = 0.0
        self.longest_stall_s = 0.0
        self.last_byte_monotonic = 0.0

    # -- kayıt yardımcıları: logger yoksa sessiz kal ------------------- #
    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warn(msg)

    def _error(self, msg):
        if self._log:
            self._log.error(msg)

    def start(self):
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self, join_timeout_s=2.0):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)
            self._thread = None

    def _run(self):
        backoff = 1.0
        while self._running:
            sock = None
            try:
                self._info(
                    f"NTRIP bağlanıyor: {self.host}:{self.port}/{self.mountpoint}"
                )
                sock, status_line, is_chunked = open_ntrip(
                    self.host, self.port, self.mountpoint, self._user, self._password
                )
                self._info(
                    f"NTRIP bağlandı: {status_line}"
                    + (" (chunked)" if is_chunked else "")
                )
                self.connected = True
                backoff = 1.0
                self._stream(sock, is_chunked)
            except PermissionError as exc:
                # Kimlik hatası tekrar denemekle düzelmez; yüksek sesle dur.
                self._error(f"NTRIP kimlik doğrulama hatası: {exc}. İstemci duruyor.")
                self.connected = False
                return
            except StallReconnect as exc:
                # Caster ayakta, yalnizca bu oturum susmus. Geri cekilme YOK:
                # beklemek 15-27 s kaybettiriyor, yeni baglanti ~0.2 s.
                self._warn(f"NTRIP durak nedeniyle yeniden baglaniliyor: {exc}")
                stall_reconnect = True
            except (OSError, ConnectionError) as exc:
                self._warn(f"NTRIP bağlantı hatası: {exc}")
                stall_reconnect = False
            except Exception as exc:
                # B55: genel yakalayıcı OLMADAN bu thread sessizce ölüyordu —
                # yeniden bağlanma denenmiyor, log'a hiçbir şey düşmüyor, tek
                # belirti status satırındaki `bağlı=False` oluyordu. Beklenmedik
                # her istisna (ör. callback'ten sızan bir hata) artık burada
                # yakalanıp normal backoff ile yeniden bağlanmaya dönüyor.
                self.unexpected_errors += 1
                self._error(
                    f"NTRIP beklenmedik hata ({type(exc).__name__}): {exc}. "
                    "Yeniden bağlanılacak."
                )
                stall_reconnect = False
            else:
                stall_reconnect = False
            finally:
                self.connected = False
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

            if not self._running:
                break
            self.reconnects += 1
            if stall_reconnect:
                backoff = 1.0
                continue
            self._warn(f"NTRIP {backoff:.0f} s sonra yeniden denenecek.")
            # Beklerken stop() çağrılırsa hemen çık
            deadline = time.monotonic() + backoff
            while self._running and time.monotonic() < deadline:
                time.sleep(0.2)
            backoff = min(backoff * 2.0, self._reconnect_max_s)

    def _stream(self, sock, is_chunked=False):
        parser = Rtcm3Parser()
        decoder = ChunkedDecoder() if is_chunked else None
        sock.settimeout(1.0)
        last_gga = 0.0
        warned_no_position = False
        self.last_byte_monotonic = time.monotonic()
        in_stall = False

        while self._running:
            now = time.monotonic()

            if now - last_gga >= self._gga_period_s:
                position = self._get_position() if self._get_position else None
                if position is None:
                    if not warned_no_position:
                        self._warn(
                            "GGA için konum yok (henüz GPS fix'i gelmedi) — "
                            "VRS konum bildirilene kadar düzeltme akıtmaz."
                        )
                        warned_no_position = True
                else:
                    warned_no_position = False
                    sock.sendall(build_gga(*position))
                    last_gga = now

            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                # Bayt gelmedi: bu bir durak mi, yoksa normal bosluk mu?
                # Sadece erken uyari. Sayaclarin TAMAMI asagida, veri geri
                # geldiginde ve TEK olcute (bosluk >= esik) gore islenir;
                # yoksa "durak=0 ama olu=3s" gibi celisen loglar cikiyor.
                bekleme = time.monotonic() - self.last_byte_monotonic
                if bekleme >= self.stall_threshold_s and not in_stall:
                    in_stall = True
                    if self._stall_since is None:
                        # Durak burada baslar; suresi veri geri gelince kesinlesir.
                        self.stalls += 1
                        self._stall_since = self.last_byte_monotonic
                    self._warn(
                        f"NTRIP akisi durdu ({bekleme:.0f} s bayt yok). TCP ayakta; "
                        "caster sustu. RTK FIXED kilidi bu duraklarda dusuyor."
                    )
                if self.stall_reconnect_s > 0 and bekleme >= self.stall_reconnect_s:
                    # Beklemek yerine kopar: yeni baglanti aninda veri veriyor.
                    # Sure BURADA eklenmez — koparma+yeniden baglanma+akisin
                    # geri gelmesi de olu zamandir, hepsi _stall_since'ten
                    # veri donunce hesaplanir (B56).
                    self.stall_reconnects += 1
                    raise StallReconnect(
                        f"{bekleme:.0f} s bayt gelmedi, oturum kopariliyor"
                    )
                continue
            if chunk == b"":
                raise ConnectionError("Caster bağlantıyı kapattı (akış kesildi)")

            simdi = time.monotonic()
            if self._stall_since is not None:
                # Durak (varsa koparma dahil) burada kapaniyor.
                bosluk = simdi - self._stall_since
                self._stall_since = None
            elif simdi - self.last_byte_monotonic >= self.stall_threshold_s:
                # recv() esikten uzun blokladi: zaman asimi yolu hic calismadi.
                bosluk = simdi - self.last_byte_monotonic
                self.stalls += 1
            else:
                bosluk = 0.0
            if bosluk > 0.0:
                self.stalled_seconds += bosluk
                self.longest_stall_s = max(self.longest_stall_s, bosluk)
                self._info(f"NTRIP akisi geri geldi ({bosluk:.0f} s durak sonrasi).")
            in_stall = False
            self.last_byte_monotonic = simdi

            self.total_bytes += len(chunk)
            if decoder is not None:
                chunk = decoder.feed(chunk)
                if decoder.finished:
                    raise ConnectionError("Caster akışı sonlandırdı (son chunk)")

            for frame in parser.feed(chunk):
                self.total_frames += 1
                self.last_frame_monotonic = time.monotonic()
                if self._on_rtcm:
                    try:
                        self._on_rtcm(frame)
                    except Exception as exc:
                        # Callback hatasi akisi oldurmemeli (B55). Ilkini
                        # bildirip sonrakileri sayacta topluyoruz.
                        self.callback_errors += 1
                        if self.callback_errors == 1:
                            self._error(
                                f"RTCM callback hatasi ({type(exc).__name__}): "
                                f"{exc}. Akis surduruluyor."
                            )

            self.crc_errors = parser.crc_errors
            self.types = parser.types
