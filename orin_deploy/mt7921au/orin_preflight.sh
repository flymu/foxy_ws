#!/bin/bash
# orin_preflight.sh — MT7921AU 到货日体检（在 Orin 上跑）
# 输出结构化报告 + 末尾 VERDICT（对照 README_MT7921AU_DAY1.md §2 选路径）
set -u
K="$(uname -r)"
HDR="/lib/modules/$K/build"
echo "================ MT7921AU PREFLIGHT $(date '+%F %T') ================"
echo "[1] 内核/L4T"
uname -r
cat /etc/nv_tegra_release 2>/dev/null || echo "(无 /etc/nv_tegra_release)"
echo
echo "[2] 现有 mt76/mt79* 模块（NVIDIA backport 迹象）"
FOUND=""
for m in mt76 mt76-usb mt76-connac-lib mt7921-common mt7921u mt7921e mt7921s; do
  p="$(modinfo -n "$m" 2>/dev/null)" || continue
  FOUND="$FOUND $m"
  echo "  $m -> $p"
done
[ -z "$FOUND" ] && echo "  （一个都没有：无 NVIDIA mt76 backport）"
for m in $FOUND; do
  modinfo "$m" 2>/dev/null | grep -E '^(version|vermagic|firmware|srcversion):' | sed "s/^/  [$m] /"
done
echo
echo "[3] 内核 headers"
if [ -f "$HDR/Makefile" ]; then
  echo "  headers 在: $HDR"
  grep -E '^CONFIG_(MT76|MT79|MAC80211|CFG80211)' "$HDR/.config" 2>/dev/null | sed 's/^/  /' || echo "  (.config 不可读)"
  echo "  mac80211 头时代探测（有无 5.12+ API）："
  grep -o 'ieee80211_get_he_6ghz_capa\|IEEE80211_HE_OPER_6GHZ\|ieee80211_iter_stations' "$HDR/include/net/mac80211.h" 2>/dev/null | sort -u | sed 's/^/    找到: /' || echo "    （没找到 5.12+ 时代的符号——大概率要走 README 路径 3/4）"
else
  echo "  !! headers 缺失: $HDR"
fi
echo
echo "[4] 工具链"
gcc --version 2>/dev/null | head -1 || echo "  !! 无 gcc（需解决编译器）"
make --version 2>/dev/null | head -1 || echo "  !! 无 make"
echo
echo "[5] USB 上的 MT7921AU"
lsusb 2>/dev/null | grep -iE '0e8d|mediatek|wireless' || echo "  !! 没看到 0e8d:7961——卡插了吗？（部分套件报成别的 VID，lsusb 全列如下）"
lsusb 2>/dev/null | sed 's/^/    /'
echo
echo "[6] 磁盘/内存"
df -h / | tail -1 | sed 's/^/  /'
free -h | head -2 | tail -1 | sed 's/^/  /'
echo
echo "[7] 现有 WiFi/NM 状态（记录 AIC 连接名，§4 切换要用）"
iw dev 2>/dev/null | grep -E 'Interface|ssid|channel' | sed 's/^/  /'
nmcli -t -f NAME,TYPE,DEVICE con show --active 2>/dev/null | sed 's/^/  /'
echo
echo "[8] 固件目录"
ls -la /lib/firmware/mediatek/ 2>/dev/null | sed 's/^/  /' || echo "  /lib/firmware/mediatek 不存在（脚本会建）"
echo
echo "================ VERDICT ================"
if modinfo -n mt7921u >/dev/null 2>&1; then
  echo "路径 0：mt7921u 已存在！只需固件 + depmod + modprobe，跳过编译（README §3 末段手动做）"
elif [ -n "$FOUND" ]; then
  echo "路径 1：NVIDIA 已 backport mt76 系但无 USB 版 → build_mt7921u.sh 编全家桶整体替换"
elif [ -f "$HDR/Makefile" ]; then
  echo "路径 2：headers 在、无现成模块 → build_mt7921u.sh 直接编；报 mac80211 API 错则转路径 3（随卡安装包）"
else
  echo "路径 3/4：headers 缺 → 先随卡安装包；再不行 README §2 路径 4（L4T kernel_src 手编）"
fi
