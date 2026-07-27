#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# ntrip_client saf mantık testleri — ROS, CAN ve internet gerektirmez.

import os

import pytest

from here4_dronecan_bridge.ntrip_client import (
    ChunkedDecoder,
    Rtcm3Parser,
    build_gga,
    crc24q,
    nmea_checksum,
)


def chunk_encode(payload, chunk_size):
    """HTTP chunked transfer-encoding ile paketler (caster'ın yaptığı gibi)."""
    out = b""
    for offset in range(0, len(payload), chunk_size):
        piece = payload[offset : offset + chunk_size]
        out += f"{len(piece):X}\r\n".encode() + piece + b"\r\n"
    return out + b"0\r\n\r\n"


def make_frame(msg_type, payload_len=10, fill=0x5A):
    """Geçerli CRC'li sentetik bir RTCM3 çerçevesi üretir."""
    payload = bytes([msg_type >> 4, (msg_type & 0x0F) << 4]) + bytes(
        (payload_len - 2) * [fill]
    )
    body = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    return body + crc24q(body).to_bytes(3, "big")


# --- NMEA GGA ------------------------------------------------------------- #


def test_gga_checksum_bagimsiz_hesapla_uyusur():
    sentence = build_gga(37.053, 35.3213, 25.0).decode("ascii")
    assert sentence.startswith("$GPGGA,")
    assert sentence.endswith("\r\n")

    body, checksum = sentence.strip()[1:].split("*")
    expected = 0
    for char in body:
        expected ^= ord(char)
    assert checksum == f"{expected:02X}"


def test_gga_derece_dakika_donusumu():
    # 37.5 derece = 37 derece 30.0 dakika -> "3730.000000"
    sentence = build_gga(37.5, 35.25, 0.0).decode("ascii")
    fields = sentence.split(",")
    assert fields[2] == "3730.000000"
    assert fields[3] == "N"
    # 35.25 derece = 35 derece 15.0 dakika -> "03515.000000" (boylam 3 haneli)
    assert fields[4] == "03515.000000"
    assert fields[5] == "E"


def test_gga_guney_bati_yarimkure():
    fields = build_gga(-33.9, -18.4, 10.0).decode("ascii").split(",")
    assert fields[3] == "S"
    assert fields[5] == "W"


def test_nmea_checksum_bilinen_deger():
    # XOR tanımı gereği aynı karakter çifti birbirini götürür
    assert nmea_checksum("AA") == "00"
    assert nmea_checksum("A") == f"{ord('A'):02X}"


# --- RTCM3 ayrıştırıcı ---------------------------------------------------- #


def test_parser_tek_seferde_coklu_cerceve():
    parser = Rtcm3Parser()
    stream = make_frame(1077) + make_frame(1005, 19) + make_frame(1230)
    frames = list(parser.feed(stream))

    assert len(frames) == 3
    assert parser.frames == 3
    assert parser.crc_errors == 0
    assert set(parser.types) == {1077, 1005, 1230}


def test_parser_onundeki_cop_atlanir():
    parser = Rtcm3Parser()
    frames = list(parser.feed(b"\x11\x22\x33" + make_frame(1077)))
    assert len(frames) == 1
    assert parser.types[1077] == 1


@pytest.mark.parametrize("chunk_size", [1, 2, 7, 13, 64, 4096])
def test_parser_tcp_parcalanmasina_dayanikli(chunk_size):
    """Caster akışı rastgele yerlerden bölünür; hiçbir çerçeve kaybolmamalı."""
    stream = make_frame(1077, 200) + make_frame(1087, 150) + make_frame(1230, 8)
    parser = Rtcm3Parser()
    collected = []
    for offset in range(0, len(stream), chunk_size):
        collected.extend(parser.feed(stream[offset : offset + chunk_size]))

    assert len(collected) == 3
    assert parser.crc_errors == 0
    assert b"".join(collected) == stream


def test_parser_bozuk_crc_reddedilir():
    parser = Rtcm3Parser()
    corrupted = bytearray(make_frame(1077))
    corrupted[-1] ^= 0xFF
    frames = list(parser.feed(bytes(corrupted)))

    assert frames == []
    assert parser.frames == 0
    assert parser.crc_errors >= 1


def test_parser_bozuk_cerceveden_sonra_senkron_yakalar():
    """Tek bozuk çerçeve akışı öldürmemeli — sonraki çerçeve yine çözülmeli."""
    corrupted = bytearray(make_frame(1077))
    corrupted[-1] ^= 0xFF
    parser = Rtcm3Parser()
    frames = list(parser.feed(bytes(corrupted) + make_frame(1230)))

    assert len(frames) == 1
    assert parser.types[1230] == 1


def test_parser_saf_cop_tamponu_sisirmez():
    """Rastgele veri gelirse tampon sınırsız büyümemeli (bellek koruması)."""
    parser = Rtcm3Parser()
    for _ in range(50):
        list(parser.feed(os.urandom(4096)))

    assert parser.frames == 0
    # Ayrıştırıcı en fazla bir tam çerçeve kadar (0xD3 + 1023 + CRC) tutar
    assert len(parser._buffer) < 2048


def test_crc24q_bozulmaya_duyarli():
    """Tek bit değişimi CRC'yi değiştirmeli."""
    data = b"\xd3\x00\x08" + b"\x12\x34\x56\x78\x9a\xbc\xde\xf0"
    original = crc24q(data)
    for bit in range(8):
        mutated = bytearray(data)
        mutated[4] ^= 1 << bit
        assert crc24q(bytes(mutated)) != original


# --- HTTP chunked transfer-encoding --------------------------------------- #
# TUSAGA-Aktif'in Trimble Ntrip Caster 5.2'si chunked yayınlıyor. Çözülmezse
# chunk başlıkları RTCM çerçevelerinin ORTASINA girer (27.07.2026 ölçümü:
# baytların %4.12'si, çerçevelerin ~%5'i bu yüzden kayboluyordu).


@pytest.mark.parametrize("chunk_size", [1, 16, 210, 1024, 4096])
def test_chunked_decoder_govdeyi_aynen_geri_verir(chunk_size):
    body = bytes(range(256)) * 20
    decoder = ChunkedDecoder()
    assert decoder.feed(chunk_encode(body, chunk_size)) == body
    assert decoder.finished


@pytest.mark.parametrize("tcp_split", [1, 3, 17, 256, 4096])
def test_chunked_decoder_tcp_parcalanmasina_dayanikli(tcp_split):
    """Chunk başlığı TCP paketi ortasında ikiye bölünebilir."""
    body = bytes(range(256)) * 8
    wire = chunk_encode(body, 210)

    decoder = ChunkedDecoder()
    out = b"".join(
        decoder.feed(wire[offset : offset + tcp_split])
        for offset in range(0, len(wire), tcp_split)
    )
    assert out == body


def test_chunked_decoder_uzunluk_uzantisini_yok_sayar():
    """ "1F;ext=deger" biçimi de geçerli (RFC 7230)."""
    decoder = ChunkedDecoder()
    assert decoder.feed(b"4;foo=bar\r\nABCD\r\n0\r\n\r\n") == b"ABCD"


def test_chunked_decoder_bozuk_basligi_yakalar():
    decoder = ChunkedDecoder()
    with pytest.raises(ConnectionError):
        decoder.feed(b"ZZZZ\r\nABCD\r\n")


def test_chunked_akista_rtcm_cerceveleri_bozulmadan_cikar():
    """Asıl regresyon: chunk sınırı çerçeve ortasına düşse bile kayıp olmamalı."""
    frames = b"".join(make_frame(t, 200) for t in (1075, 1085, 1095, 1125))
    wire = chunk_encode(frames, 210)  # 210 = sahada gözlenen chunk boyutu

    # De-chunk EDİLMEDEN: çerçeveler bozulur
    ham = Rtcm3Parser()
    list(ham.feed(wire))

    # De-chunk EDİLEREK: hepsi çıkar
    decoder, parser = ChunkedDecoder(), Rtcm3Parser()
    cozulen = list(parser.feed(decoder.feed(wire)))

    assert len(cozulen) == 4
    assert parser.crc_errors == 0
    assert set(parser.types) == {1075, 1085, 1095, 1125}
    assert ham.frames < 4, "de-chunk edilmeden kayıp olmalıydı (regresyon kanıtı)"
