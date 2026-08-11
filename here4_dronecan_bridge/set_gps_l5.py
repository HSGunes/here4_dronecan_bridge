#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# Here4'ün GPS L5 sağlık ezmesini (GPS_DRV_OPTIONS bit 5) DroneCAN üzerinden
# aç/kapat.
#
# NEDEN: NEO-F9P datasheet'i "GPS L5 signals are pre-operational and not used
# by default" diyor — L5 uyduları navigasyon mesajında kendilerini sağlıksız
# ilan ettiği için alıcı onları ATIYOR. ArduPilot'ta bunu ezen anahtar
# GPS_DRV_OPTIONS bit 5 (GPSL5HealthOverride) -> CFG-SIGNAL-L5_HEALTH_OVR.
#
# ETKİSİ: Here4 L1/L5 bir alıcı (L2 alamıyor). Bit 5 kapalıyken GPS, RTK
# belirsizlik çözümüne yalnız L1 ile katılıyor — yani en kalabalık takımyıldız
# TEK FREKANS. TUSAGA düzeltmeleri GPS L5'i (5X) gönderiyor, alıcı reddediyor.
# Bit 5 açıkken GPS de çift frekans olur ve FIXED'e yakınsama hızlanmalı.
#
# UYARI: u-blox bu ayarı "safety-of-life sistemlerinde önermiyoruz" diyor;
# gerçekten bozuk bir L5 sinyali çözüme girebilir. A/B ölçmeden kalıcı yapma.
#
# Kullanım:
#   ros2 run here4_dronecan_bridge set_gps_l5 --show      # mevcut değeri oku
#   ros2 run here4_dronecan_bridge set_gps_l5 --enable    # bit 5'i aç
#   ros2 run here4_dronecan_bridge set_gps_l5 --disable   # bit 5'i kapat
#
# DİKKAT: köprü node'u çalışırken ÇALIŞTIRMA — aynı CAN üzerinde ikinci bir
# DroneCAN düğümü açar. Önce köprüyü durdur, parametreyi yaz, sonra başlat.

import argparse
import sys
import time

import dronecan

PARAM = "GPS_DRV_OPTIONS"
L5_BIT = 5
L5_MASK = 1 << L5_BIT  # 32


def _istek(node, target, request, zaman_asimi=4.0):
    """Tek bir GetSet servis çağrısı yapar, yanıtı döndürür (yoksa None)."""
    sonuc = {}
    node.request(request, target, lambda e: sonuc.__setitem__("e", e))
    son = time.time() + zaman_asimi
    while time.time() < son and "e" not in sonuc:
        try:
            node.spin(timeout=0.02)
        except Exception:
            pass
    return sonuc.get("e")


def oku(node, target):
    """GPS_DRV_OPTIONS değerini adıyla sorgular."""
    req = dronecan.uavcan.protocol.param.GetSet.Request(
        name=PARAM.encode(), index=0
    )
    event = _istek(node, target, req)
    if event is None:
        return None
    ad = bytes(bytearray(event.response.name)).decode(errors="replace")
    if ad != PARAM:
        return None
    return int(event.response.value.integer_value)


def yaz(node, target, deger):
    """GPS_DRV_OPTIONS'a yeni değer yazar, cihazın döndürdüğü değeri verir."""
    req = dronecan.uavcan.protocol.param.GetSet.Request(
        name=PARAM.encode(),
        value=dronecan.uavcan.protocol.param.Value(integer_value=int(deger)),
    )
    event = _istek(node, target, req)
    if event is None:
        return None
    return int(event.response.value.integer_value)


def kaydet(node, target):
    """Parametreleri kalıcı belleğe yazdırır (ExecuteOpcode SAVE)."""
    req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request(
        opcode=dronecan.uavcan.protocol.param.ExecuteOpcode.Request().OPCODE_SAVE
    )
    event = _istek(node, target, req, zaman_asimi=6.0)
    return bool(event and event.response.ok)


def main():
    ap = argparse.ArgumentParser(
        description="Here4 GPS L5 sağlık ezmesi (GPS_DRV_OPTIONS bit 5)"
    )
    ap.add_argument("--can", default="can0")
    ap.add_argument("--target", type=int, default=125, help="Here4 node ID")
    ap.add_argument("--node-id", type=int, default=40, help="bu aracın node ID'si")
    grup = ap.add_mutually_exclusive_group(required=True)
    grup.add_argument("--show", action="store_true")
    grup.add_argument("--enable", action="store_true")
    grup.add_argument("--disable", action="store_true")
    args = ap.parse_args()

    try:
        node = dronecan.make_node(args.can, node_id=args.node_id)
    except Exception as exc:
        print(f"[HATA] CAN açılamadı ({args.can}): {exc}", file=sys.stderr)
        return 1

    try:
        mevcut = oku(node, args.target)
        if mevcut is None:
            print(
                f"[HATA] node {args.target}'ten {PARAM} okunamadı. "
                "Here4 bus'ta mı, köprü node'u kapalı mı?",
                file=sys.stderr,
            )
            return 1
        print(f"{PARAM} = {mevcut}   (L5 ezmesi: {'AÇIK' if mevcut & L5_MASK else 'KAPALI'})")
        if args.show:
            return 0

        hedef = (mevcut | L5_MASK) if args.enable else (mevcut & ~L5_MASK)
        if hedef == mevcut:
            print("Değer zaten istenen halde, yazılmadı.")
            return 0

        yeni = yaz(node, args.target, hedef)
        if yeni is None:
            print("[HATA] yazma isteğine yanıt gelmedi.", file=sys.stderr)
            return 1
        if yeni != hedef:
            print(
                f"[HATA] cihaz {hedef} yerine {yeni} bildirdi — yazma reddedilmiş.",
                file=sys.stderr,
            )
            return 1
        print(f"{PARAM} -> {yeni}   (L5 ezmesi: {'AÇIK' if yeni & L5_MASK else 'KAPALI'})")

        if kaydet(node, args.target):
            print("Kalıcı belleğe kaydedildi.")
        else:
            print(
                "[UYARI] SAVE onayı gelmedi — değer yeniden başlatmada kaybolabilir.",
                file=sys.stderr,
            )
        print("Etkili olması için Here4'ü yeniden başlat (güç kes/ver).")
        return 0
    finally:
        node.close()


if __name__ == "__main__":
    sys.exit(main())
