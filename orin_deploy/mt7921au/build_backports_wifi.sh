#!/bin/bash
# build_backports_wifi.sh — 在 Orin 上用 backports-6.1.145 构建 mt7921u（2026-11-12 路径4落地版）
#
# 背景：pinned mt76(bdf8ea71, 2023-09) 直接编不过 tegra 5.10.104——tegra 自带的 mac80211
# 是 ~5.11 时代的（SAR specs / sta_set_decap_offload / ieee80211_disconnect / HE 枚举改名
# 全都不在）。backports 是官方兼容层：整体构建 cfg80211+mac80211+mt76 一家人，装进
# updates/ 后 depmod 优先于内核自带版本（同目录 shadow，不删内核文件）。
#
# 用法：  bash build_backports_wifi.sh           # 默认装 firmware/linux-firmware-2025-09（与 2025 版驱动同代）
#         bash build_backports_wifi.sh --fw old  # 换 firmware/mt76-pinned-2023-09 那对
# 远程跑：printf '<sudo密码>\n' | ssh go2_lan 'sudo -S bash /home/unitree/mt7921au/build_backports_wifi.sh'
#         （sudo -S 从 stdin 读密码；整脚本以 root 跑——免 sudo 无 tty 时票据分桶的坑，
#           编内核模块不碰 DDS，root 安全；结尾自动 chown 还给 unitree）
# 前置：  backports-6.1.145-1.tar.xz 在本目录；内核 headers 在；flex/bison 已查在；卡已枚举（lsusb）。
set -e
set -o pipefail   # make|tee 的失败别被 tee 吞
HERE="$(cd "$(dirname "$0")" && pwd)"
K="$(uname -r)"
HDR="/lib/modules/$K/build"
FWDIR="$HERE/firmware/linux-firmware-2025-09"
[ "${1:-}" = "--fw" ] && [ "${2:-}" = "old" ] && FWDIR="$HERE/firmware/mt76-pinned-2023-09"

[ -f "$HERE/backports-6.1.145-1.tar.xz" ] || { echo "!! 缺 backports-6.1.145-1.tar.xz"; exit 1; }
[ -f "$HDR/Makefile" ] || { echo "!! 缺内核 headers: $HDR"; exit 1; }
[ -f "$FWDIR/WIFI_MT7961_patch_mcu_1_2_hdr.bin" ] || { echo "!! 缺固件: $FWDIR"; exit 1; }
# 卡状态只提示不拦截：驱动装好后【热插拔重插一次】也会立即认领（11-13 实测：无驱动
# 认领时这卡会像 AIC 一样 wedge 掉线，重插即活）——所以顺序 = 装驱动 → modprobe → 重插。
lsusb | grep -q "0e8d:7961" || echo "⚠ lsusb 暂无 0e8d:7961——驱动装完后【重插一次卡】即可热插拔认领"

echo "== [1/5] 解包（backports 包有顶层目录，要 --strip-components=1）=="
rm -rf "$HERE/backports-build"
mkdir -p "$HERE/backports-build"
tar xf "$HERE/backports-6.1.145-1.tar.xz" -C "$HERE/backports-build" --strip-components=1
cd "$HERE/backports-build"

echo "== [2/5] 精简 config：只编 cfg80211+mac80211+mt7921u 一家人 =="
make defconfig-wifi 2>&1 | tail -3
# 全部模块关掉，再只开三件（MT7921U 一开会自动 select MT7921_COMMON→MT76_CONNAC_LIB→
# MT76_CORE、MT76_USB——Kconfig 里它们都是无 prompt 的 tristate，靠 select 传染成 m）
sed -i 's/^\(CPTCFG_[A-Z0-9_]*\)=m$/\1=n/' .config
for sym in CFG80211 MAC80211 MT7921U; do
  sed -i -e "s/^CPTCFG_${sym}=n\$/CPTCFG_${sym}=m/" \
         -e "s/^# CPTCFG_${sym} is not set/CPTCFG_${sym}=m/" .config
done
make olddefconfig >/dev/null
echo "-- 裁剪后模块清单："
grep -E "^CPTCFG_.*=m$" .config || true
MISSING=""
for sym in CFG80211 MAC80211 MT7921U MT7921_COMMON MT76_USB MT76_CONNAC_LIB MT76_CORE; do
  grep -q "^CPTCFG_${sym}=m$" .config || MISSING="$MISSING $sym"
done
if [ -n "$MISSING" ]; then
  echo "!! 精简 config 失败，缺:$MISSING"
  echo "   兜底：删掉本段 sed/olddefconfig，直接用完整 make defconfig-wifi 全量构建（慢但稳）"
  exit 1
fi

echo "== [3/5] 编译（-j$(nproc)，cfg80211+mac80211+mt76 全家 ≈ 几分钟）=="
# 全量日志落盘；失败才回显尾部，避免远程刷屏
if ! make -j"$(nproc)" > "$HERE/backports_build.log" 2>&1; then
  echo "!! 编译失败，尾部 40 行（全文 $HERE/backports_build.log）："
  tail -40 "$HERE/backports_build.log"
  exit 1
fi
echo "-- 编译 OK，产物："
find . -name "*.ko" | sort

echo "== [4/5] 安装（updates/ 目录 shadow 内核自带版，不删内核文件）+ 固件 + depmod =="
# make install 尾步 update-initramfs.sh 会调 update-grub——Orin/L4T 用 extlinux 没有 grub，
# 该步必失败但模块/黑名单/depmod 早已完成 → 不能让 set -e 在这里翻车，改显式验证产物。
sudo make install > "$HERE/install.log" 2>&1 || echo "   (make install 尾步失败，若下面验证过=只是 update-grub 不存在，忽略)"
for m in cfg80211 mac80211 mt76 mt76-usb mt76-connac-lib mt7921-common mt7921u; do
  found="$(find "/lib/modules/$K/updates" -name "$m.ko*" | head -1)"
  [ -n "$found" ] || { echo "!! updates/ 里没有 $m.ko —— 安装未完成（看 $HERE/install.log）"; exit 1; }
  echo "  OK $found"
done
sudo cp "$FWDIR"/*.bin /lib/firmware/mediatek/
sudo depmod -a "$K"

echo "== [5/5] modprobe + dmesg =="
# AIC 卡已拔、MT7921 之前没人认领，理论上无 WiFi 栈在跑——但保险起见先卸内核自带的旧栈
# （同名模块一旦已加载，modprobe 会直接用旧货，我们的新 mt7921u 会符号对不上）。
for m in mt7921u mt7921-common mt76-connac-lib mt76-usb mt76-sdio mt76 mac80211 cfg80211; do
  sudo rmmod "$m" 2>/dev/null || true
done
if lsmod | grep -qE "^(cfg80211|mac80211|mt76)"; then
  echo "!! 旧 WiFi 栈卸不掉（有东西在用），硬上会绑错版本——先查 lsmod 里谁占着再重跑"
  lsmod | grep -E "^(cfg80211|mac80211|mt76)"
  exit 1
fi
sudo modprobe mt7921u
sleep 5
dmesg | grep -iE "mt7921|mt76|WIFI_MT7961|cfg80211|firmware" | tail -20
echo
echo "== 验证 =="
iw dev
ls /sys/class/net/
echo
echo "成功的标志：dmesg 无 firmware 报错 + iw dev 多出一张卡。"
echo "固件报错就换另一对重跑本脚本加 --fw old。"

# 若整脚本以 root 跑（远程 sudo -S bash），把本目录属主还给 unitree，免得下次删不掉
if [ "$(id -u)" = "0" ]; then
  chown -R unitree:unitree "$HERE" 2>/dev/null || true
fi