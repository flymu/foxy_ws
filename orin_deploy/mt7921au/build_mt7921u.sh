#!/bin/bash
# build_mt7921u.sh — 在 Orin 上编译安装 mt7921u（mt76 pinned bdf8ea71，5.10 实证版）
# 用法：  bash build_mt7921u.sh           # 默认装 firmware/mt76-pinned-2023-09 固件
#         bash build_mt7921u.sh --fw new  # 换 firmware/linux-firmware-2025-09 那对固件
# 前置：  orin_preflight.sh VERDICT = 路径 1/2（headers 在）；卡已插（lsusb 见 0e8d:7961）
set -e
set -o pipefail   # 否则 make|tee 的失败被 tee 吞掉，重试分支永远不触发
HERE="$(cd "$(dirname "$0")" && pwd)"
K="$(uname -r)"
HDR="/lib/modules/$K/build"
FWDIR="$HERE/firmware/mt76-pinned-2023-09"
[ "${1:-}" = "--fw" ] && [ "${2:-}" = "new" ] && FWDIR="$HERE/firmware/linux-firmware-2025-09"

[ -f "$HERE/mt76-bdf8ea71.tar.gz" ] || { echo "!! 缺源码包 mt76-bdf8ea71.tar.gz"; exit 1; }
[ -f "$HDR/Makefile" ] || { echo "!! 缺内核 headers: $HDR（README §2 路径 3/4）"; exit 1; }
[ -f "$FWDIR/WIFI_MT7961_patch_mcu_1_2_hdr.bin" ] || { echo "!! 缺固件: $FWDIR"; exit 1; }

echo "== [1/5] 解包 pinned 源 =="
rm -rf "$HERE/mt76-build"
mkdir -p "$HERE/mt76-build"
# 注意：git archive 出的包没有顶层目录（成员直接就是 Makefile、mt7921/、…），
# 所以解包【不加】--strip-components=1，加了会把目录拍扁、根 Makefile 直接消失。
tar xf "$HERE/mt76-bdf8ea71.tar.gz" -C "$HERE/mt76-build"
cd "$HERE/mt76-build"

# ---- tegra 5.10 缺 mtk_wed.h 桩（11-12 实测首编译 22 错全是这一个）----
# <linux/soc/mediatek/mtk_wed.h> 是 5.16 才进主线的；mt76-2023-09 无条件 include 它。
# 真实 WED 调用全在 #ifdef CONFIG_NET_MEDIATEK_SOC_WED 里（tegra 上该配置关），
# 无防护的引用只有 3 处（dma.c:812-813 active/detach、mmio.c:77-79 active/irq_set_mask）
# ——上游头文件在 config-off 分支提供的正是这三个 no-op 桩，这里原样复刻。
cat > mtk_wed_stub.h <<'STUBEOF'
#ifndef __MTK_WED_STUB_H__
#define __MTK_WED_STUB_H__
#include <linux/kernel.h>
#include <linux/types.h>

struct mtk_wed_device { };

static inline bool mtk_wed_device_active(struct mtk_wed_device *dev)
{
	return false;
}

static inline void mtk_wed_device_detach(struct mtk_wed_device *dev)
{
}

static inline void mtk_wed_device_irq_set_mask(struct mtk_wed_device *dev,
					       u32 mask)
{
}
#endif
STUBEOF
sed -i 's|#include <linux/soc/mediatek/mtk_wed.h>|#include "mtk_wed_stub.h"|' mt76.h
grep -q mtk_wed_stub.h mt76.h || { echo "!! mt76.h 桩替换失败"; exit 1; }

# 只开 USB 路线要的 5 个模块；**显式关掉其余全家**——M= 外部构建会继承运行内核 .config
# 的 CONFIG 符号（WS 6.8 的 .config 里 mt7615/mt76x0 等都是 m，不关就会误编不需要的
# 老驱动，它们的 API 兼容性和我们要的无关，白报错）。
BUILD_FLAGS="CONFIG_MT76_USB=m CONFIG_MT76_CONNAC_LIB=m CONFIG_MT7921_COMMON=m CONFIG_MT7921U=m \
  CONFIG_MT76_SDIO= CONFIG_MT76x02_LIB= CONFIG_MT76x02_USB= CONFIG_MT76x0_COMMON= \
  CONFIG_MT76x2_COMMON= CONFIG_MT7603E= CONFIG_MT7615_COMMON= CONFIG_MT7915E= \
  CONFIG_MT7921E= CONFIG_MT7921S= CONFIG_MT7925_COMMON= CONFIG_MT7996E= \
  CONFIG_NL80211_TESTMODE= CONFIG_PCI="
echo "== [2/5] 编译（tegra 原生 mac80211 头，第 1 次带 -Werror）=="
if ! make -C "$HDR" M="$PWD" -j"$(nproc)" modules $BUILD_FLAGS 2>&1 | tee "$HERE/build.log"; then
  echo "== 编译失败；去 -Werror 重试（老内核+新 gcc 常见坑）=="
  sed -i 's/ -Werror//' Makefile
  make -C "$HDR" M="$PWD" -j"$(nproc)" modules $BUILD_FLAGS 2>&1 | tee -a "$HERE/build.log" || {
    echo "!! 两次都失败。若 build.log 是 mac80211/cfg80211 API 报错（unknown field/implicit decl）"
    echo "   = tegra 的 mac80211 太老 → 走 README §2 路径 3（随卡安装包）或路径 4。"
    exit 1; }
fi

echo "== [3/5] 找 5 个 .ko =="
KOS=""
for m in mt76 mt76-usb mt76-connac-lib mt7921-common mt7921u; do
  ko="$(find "$HERE/mt76-build" -name "$m.ko" | head -1)"
  [ -n "$ko" ] || { echo "!! 没编出 $m.ko"; exit 1; }
  echo "  $ko"
  KOS="$KOS $ko"
done

echo "== [4/5] 装固件 + 模块 + depmod（要 sudo）=="
sudo mkdir -p /lib/firmware/mediatek
sudo cp "$FWDIR"/*.bin /lib/firmware/mediatek/
sudo mkdir -p "/lib/modules/$K/updates"
sudo cp $KOS "/lib/modules/$K/updates/"
sudo depmod -a "$K"

echo "== [5/5] modprobe + dmesg =="
sudo modprobe mt7921u
sleep 3
dmesg | grep -iE 'mt7921|mt76|WIFI_MT7961' | tail -15
echo
echo "== 验证 =="
iw dev
echo
echo "全部 OK 的标志：dmesg 无 firmware 报错 + iw dev 多出一张卡。"
echo "固件报错就换另一对重跑本脚本加 --fw new。"
