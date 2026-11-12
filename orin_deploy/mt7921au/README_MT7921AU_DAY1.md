# MT7921AU 5G 网卡 — 到货日操作手册（Day-1 Runbook）

> 背景（11-08 定案）：Orin 的 AIC8800DC **2.4G 单带扛不动 LIO 输入流**——CustomMsg 每订阅者
> 32Mbps 级 + 200Hz IMU 已贴满 20MHz 信道（survey busy 50~70%），LIO 8 深队列丢帧（日志
> `(lag!)` canary）→ 位姿飞。真修 = MT7921AU 上 5G（littlered_5G，ch44）。
>
> ⚠️ **Orin 内核 5.10.104-tegra < mt7921u 进主线版本（5.12）——不是即插即用**。
> 本手册把到货日的每一步和所有未知数列清楚，照顺序走即可。

## ✅ Day-1 战报（2026-11-13 凌晨实弹完成，驱动+5G 全通）

**最终路线：路径 4 = backports-6.1.145**（下面 §2 的决策树走完，路径 1 证死，详见末尾教训）。

实测结果（狗趴窝位，5220MHz/ch44）：

| 项 | 结果 | 判据 | 判定 |
|---|---|---|---|
| 驱动 | backports 编译 7 模块一次过，`mt7921u` 认领、固件零报错 | dmesg 干净 | ✅ |
| 协商速率 | **1080.6 Mbit/s @ 80MHz HE-MCS10 双流（WiFi-6）** | — | ✅（2.4G 是 20MHz 一位数） |
| 桌旁 RSSI | **-53 dBm** | ≥ -60dBm | ✅ |
| UDP 60M Orin→WS（雷达流方向） | **60.0 Mbit/s，丢 13/51792 = 0.025%** | ≥45Mbps、≤5% | ✅ |
| UDP 60M WS→Orin | 60.0 Mbit/s，**0%** | 同上 | ✅ |
| TCP 双向 | ~197–213 Mbit/s | 参考 | ✅ |
| IP | DHCP 自然发 **.141**（AIC 租约过期后同池回发），`go2_wifi` 别名零改动 | .141 | ✅ |
| 时钟 | 2.4G 一连上 NTP 就同步好了（synchronized: yes） | 清单 1.0 | ✅ |

**待办两门**：§5 覆盖徒步（要牵狗走一圈）+ LIO 实弹门（起栈看 `lag!` 绝迹/fitness/IMU），
过了才解冻自主巡逻。

**Day-1 踩坑档案（下次别再踩）**：

1. **卡无驱动认领会 wedge**（和 AIC 同款病）：狗断电重启后卡不枚举（lsusb 无 0e8d:7961，
   dmesg 零记录）——拔下重插立刻活。驱动装好后 udev 经 modalias 自动认领（已验
   `modules.alias` 有 `usb:v0E8Dp7961… → mt7921u`），此后冷启动无需再管。
2. **路径 1（pinned mt76 bdf8ea71 直编 tegra headers）死路**：tegra 自带 mac80211 ≈ 5.11 代，
   缺 `set_sar_specs`/`sta_set_decap_offload`/`ieee80211_disconnect`/SAR 结构/HE 枚举改名
   （WS 6.8 直编同源码成功，Orin 22 错全这几样）——桩头文件救不了成员缺位，别再试。
3. **backports 正解**：`build_backports_wifi.sh`（本目录）+ `backports-6.1.145-1.tar.xz`
   （不在仓库，见 §0 注）。只编 cfg80211+mac80211+mt76 一家（`make defconfig-wifi` 后
   sed 全关再开三件、`olddefconfig` 让 select 链自动补齐），装 updates/ shadow。
4. **make install 尾步 update-grub 在 L4T 必失败**（extlinux 无 grub）——模块/黑名单/depmod
   早已完成，忽略即可（脚本已显式验证产物代替信任退出码）。
5. **L4T 特有 `firmware_class.path=/etc/firmware` 优先于 /lib/firmware**——查过 /etc/firmware
   无 mediatek 副本，无遮蔽；若日后往里放固件注意它会赢。
6. **nmcli 走 ssh 无 polkit 权限**（not authorized）——一律 `sudo nmcli`。
7. **连 5G 不用手输密码**：`nmcli con clone littlered littlered_5G` 把 2.4G 老配置的 psk
   原地复制，再改 `ssid/band=a/priority=10/powersave=2`（省电必须关，AIC 4.5Mbps 血案教训）。

## 0. 本目录清单（已备好，无需再下载）

| 文件 | 用途 |
|---|---|
| `build_backports_wifi.sh` | **Day-1 实际成功的编译脚本**（backports 6.1.145 → cfg80211+mac80211+mt7921u 一家，装 updates/） |
| `mt76-bdf8ea71.tar.gz` | mt76 pinned 源（**路径 1 已证死**，留档勿再用） |
| `firmware/mt76-pinned-2023-09/` | 备选固件对（脚本 `--fw old` 切换用） |
| `firmware/linux-firmware-2025-09/` | **Day-1 实装固件**：加载零报错（内部 Build Time 印 2023-11-09，即 linux-firmware 包含版） |
| `orin_preflight.sh` | 插卡后在 Orin 上跑，出结构化体检报告 → **报告决定走下面哪条路径**（Day-1 实测正确判到路径 1 环境） |
| `build_mt7921u.sh` | 路径 1/2 脚本（**tegra mac80211 缺 API 已证编不过，留档**） |
| `wifi_littlered_5G.nmconnection` | NM 模板（**密码占位符，真实密码绝不进仓库/绝不回传 WS**；Day-1 实际用 `nmcli con clone` 法=战报第 7 条，此文件留作参考） |
| `rssi_walk_test.sh` | 5G 覆盖徒步测试（RSSI + ping RTT 落 CSV，**待跑**） |

> **backports tarball 不在仓库**（.gitignore 排除二进制）：`backports-6.1.145-1.tar.xz`
> 在 WS `/home/mu/workspace/`、Orin `/home/unitree/mt7921au/`（编译产物 351M 已清）。
> 重取：`curl -LO https://mirrors.edge.kernel.org/pub/linux/kernel/projects/backports/stable/v6.1.145/backports-6.1.145-1.tar.xz`
> 校验 sha256 `05f2c39f1be0aab49014a6e27a9492ce7ed0491f552db8a07f3ba0de33b9317d`
> （backports.wifi.kernel.org 的 DNS/WebFetch 都不通，用 mirrors.edge.kernel.org）。

需要编译的模块 = backports 一家：`compat.ko`、`cfg80211.ko`、`mac80211.ko`、`mt76.ko`、
`mt76-usb.ko`、`mt76-connac-lib.ko`、`mt7921-common.ko`、`mt7921u.ko`
（**别混用 NVIDIA 自带 mt76/mac80211**——同名模块靠 updates/ 优先级整体 shadow）。

## 1. 先在 WS 上验卡（零编译，10 分钟，把"卡好不好"和"Orin 编译难"分开）

WS 内核 6.8 **自带 mt7921u**，插 WS 就能活：

- [ ] 插入 WS USB 口 → `dmesg | tail -20` 应见 mt7921u 加载、请求 `WIFI_MT7961_*` 固件成功
- [ ] 出新接口（`iw dev` 多一张卡）→ `nmcli dev wifi` 能扫到 littlered_5G（ch44）
- [ ] 顺手留档：`iw dev <新if> scan \| grep -A2 littlered_5G`（信号强度）、
      `iw list \| grep -A30 Frequencies`（支持哪些 5G 信道）
- [ ] 测完断开 WS 上的临时连接（别让它 autoconnect 抢线）：
      `nmcli con show` 找到新卡的连接 → `nmcli con mod "<名>" connection.autoconnect no`

**WS 能活 = 卡是好的 + 固件文件名/内容验证通过**。后面全是 Orin 侧的事。

## 2. Orin 体检 → 选路径

> **Day-1 后记**：这张决策树被实战验证到"路径 1 证死、路径 4 是正解"。Orin 环境已装好驱动
> （updates/ + modules.alias），**换卡/重装时直接跑 §3 的 backports 脚本即可**，本节留作
> 以后新内核/新卡的决策参考。

```bash
# 从 WS 把整个目录推过去（11-13 起走有线更稳：go2_lan；scp -r 不大没问题）
scp -r /home/mu/foxy_ws/orin_deploy/mt7921au go2_lan:/home/unitree/mt7921au
ssh go2_lan 'bash /home/unitree/mt7921au/orin_preflight.sh' | tee orin_preflight_$(date +%m%d).txt
```

体检报告末尾会打 VERDICT。对照选路：

| VERDICT | 路径 | 做法 |
|---|---|---|
| **mt7921u 已在** (`modinfo mt7921u` 有) | **路径 0（最好）** | NVIDIA 已把 USB 版 backport → 只需装固件 + `depmod` + `modprobe`，跳过编译 |
| 有 mt76/mt79* 但无 mt7921u | ~~路径 1~~ **已证死** | pinned bdf8ea71 直编 tegra headers 编不过——tegra 自带 mac80211 ≈5.11 代缺 SAR/decap-offload API（11-13 实弹 22 错）。**直接走路径 4** |
| 无 mt76 但 headers 在 | ~~路径 2~~ **同死因** | 同上。**直接走路径 4** |
| 编译报 mac80211 API 错 | **路径 3** | 用**随卡光盘/官方下载的 Linux 安装包**（Comfast/Brostrend/Alfa 的 MT7921AU 套件通常带 aarch64 安装脚本和旧内核兼容层）——装完用我们的固件副本复核 |
| **路径 4（Day-1 实战正解）** | ✅ **backports-6.1.145** | `build_backports_wifi.sh`（见 §3）——官方兼容层整体 backport cfg80211+mac80211，对 5.10 支持完好（5.10 LTS 本来就是它的头号目标内核） |

## 3. 编译安装

**实战正解（路径 4，11-13 一次通过）**：

```bash
# 前置：backports tarball 已在 /home/unitree/mt7921au/（WS 端 /home/mu/workspace/ 的副本推送）
scp /home/mu/workspace/backports-6.1.145-1.tar.xz go2_lan:/home/unitree/mt7921au/
# 整脚本以 root 跑（sudo -S 从 stdin 读密码，免无 tty 时票据分桶坑；编内核模块不碰 DDS）
printf '<sudo密码>\n' | ssh go2_lan 'sudo -S bash /home/unitree/mt7921au/build_backports_wifi.sh'
```

脚本自动：解包（backports 有顶层目录，**要** `--strip-components=1`，与 git archive 相反）→
`make defconfig-wifi` → sed 全关只开 CFG80211/MAC80211/MT7921U（select 链自动补齐
mt76/mt76-usb/mt76-connac-lib/mt7921-common）→ `make -j6` → `sudo make install`（尾步
update-grub 在 L4T 必失败，模块/黑名单/depmod 早已完成；脚本显式验证 7 个 .ko 在
updates/）→ 装固件 → `depmod -a` → 卸旧栈（若有）→ `modprobe mt7921u` → dmesg + `iw dev`。

**卡此时若 wedge 了（lsusb 无 0e8d:7961）：拔下重插一次**——驱动已注册，热插拔立即认领。

验证（脚本末尾也会打）：

```bash
printf '<sudo密码>\n' | ssh go2_lan 'sudo -S dmesg 2>/dev/null | grep -iE "mt7921|WIFI_MT7961" | tail -5'
ssh go2_lan 'iw dev'        # 多出 wlan0 即通
```

若 `dmesg` 报固件错 → `build_backports_wifi.sh --fw old` 换 2023-09 那对重跑。

<details>
<summary>路径 1/2 旧脚本（留档，tegra mac80211 API 缺位已证编不过）</summary>

```bash
ssh go2_wifi 'bash /home/unitree/mt7921au/build_mt7921u.sh' 2>&1 | tee orin_build_$(date +%m%d).log
```
</details>

脚本自动做：解包 pinned 源 → `make -C /lib/modules/$(uname -r)/build M=... modules` →
失败自动去 `-Werror` 重试一次 → 装固件（首选 mt76-pinned 对）→ 5 个 .ko 装进
`/lib/modules/$(uname -r)/updates/` → `depmod -a` → `modprobe mt7921u` → 打 dmesg 尾部。

验证（脚本末尾也会打）：

```bash
ssh go2_wifi 'dmesg | grep -iE "mt7921|mt76" | tail -10'   # 固件加载无报错
ssh go2_wifi 'iw dev'                                       # 多出一张无线卡
```

若 `dmesg` 报固件错 → 换 `firmware/linux-firmware-2025-09/` 那对重装重试（脚本有 `--fw new`）。

## 4. 连 littlered_5G + IP 切换（Day-1 实战版）

**密码只进 Orin 的 `/etc/NetworkManager/system-connections/`，绝不写进仓库/不回传 WS。**
⚠️ ssh 里 nmcli 无 polkit 权限，一律 `sudo nmcli`。

Day-1 实际操作（AIC 已物理拔除，无回退卡；密码用 clone 法原地复用，全程不外露）：

```bash
# 1) 克隆 2.4G 老配置（psk 原地复制不外露）→ 改 SSID/锁 5G 波段/提优先级/关省电
printf '<sudo密码>\n' | ssh go2_lan 'sudo -S nmcli con clone littlered littlered_5G 2>/dev/null && \
  sudo nmcli con mod littlered_5G 802-11-wireless.ssid littlered_5G 802-11-wireless.band a \
    connection.autoconnect-priority 10 802-11-wireless.powersave 2'
# 2) 重扫确认 littlered_5G 可见（5G 扫描要 ~10s，别用 6s 快扫的空结果下结论）
# 3) 切换
ssh go2_lan 'sudo nmcli dev disconnect wlan0; sleep 2; sudo nmcli con up littlered_5G'
# 4) 验证
ssh go2_lan 'iw dev wlan0 link; ip -4 addr show wlan0 | grep inet'
```

- **IP**：无需手工配置——AIC 租约过期后 DHCP 把 **.141 同池回发**给 MT7921 的 MAC，
  `go2_wifi` 别名零改动（11-13 实测）。建议路由器后台把 .141 绑定到
  `90:de:80:3e:2a:a9`（防日后长租约过期被别的设备抢走）。
- **优先级**：littlered_5G priority=10 > littlered(2.4G) 默认 0——5G 可见必选 5G，
  5G 掉线自动退 2.4G 保住手动遥控链路（veto 不过 WiFi，遥控不受影响；2.4G 只跑不了 LIO）。
- **省电必须关**（powersave=2）：AIC 时代省电把 TCP 压到 4.5Mbps 的血案教训。

NTP 注意：WiFi 一连上 `timedatectl` 应照样 `synchronized: yes`（11-13 实测 2.4G 连上几分钟内
就同步好了，切 5G 不变）——上电清单 1.0 的对时要求不变。

## 5. 验收门（全过才恢复自主巡逻）

- [x] **桌旁基线（11-13 PASS）**：RSSI **-53dBm** @ ch44/80MHz/HE-MCS10 双流
      （1080.6 Mbit/s 协商）。吞吐（iperf3；WS 侧无 iperf3、容器未起，用了
      `apt download iperf3 libiperf0` + `dpkg -x` 在 /tmp 免 sudo 起的 server）：
      **UDP 60M Orin→WS（雷达流方向）：60.0Mbit/s，丢 13/51792=0.025%**；
      UDP 60M WS→Orin：60.0Mbit/s **0%**；TCP 双向 ~197–213Mbit/s。
      判据（丢包 ≤5%、≥45Mbps）大幅通过——2.4G 时代同测试丢包贴 2.5% 还顶着 busy 50~70%。
- [ ] **覆盖徒步**：`ssh go2_wifi 'bash /home/unitree/mt7921au/rssi_walk_test.sh'`
      牵狗沿巡逻线走一圈，Ctrl-C 结束出 min/p10/median；**沿途 RSSI ≥ -65dBm**，掉进
      -70s 的点要记录位置（后续要么挪路由器要么记为减速/中止区）
- [ ] **LIO 实测（决定性）**：起巡检栈（上电清单 2.x 全照走，含 2.2b 铁律）：
      - `grep -E "lag!|blind" /tmp/live_stack/lio.log` **为空**（这个 canary 在 2.4G
        时代每次饱和都出现，5G 健了必须绝迹）
      - fitness ≤ 0.02 且稳
      - 容器端 IMU 探针 ≥190Hz、无 >50ms 断档（病A canary）
      - G2 30min 浸泡 + G3 蠕行 + G4 全过 → 恢复自主巡逻

## 6. 失败兜底

5G 也扛不住（路由器远/墙厚）→ 选项按序：littlered_5G 路由器侧查 40MHz 能否开（AIC 案已证
2.4G 单带是死路，5G 的 40MHz 是下一个杠杆）→ 移路由器 → 降 CustomMsg 帧率/点数。
**手动遥控全程不受影响**（veto 吃 Orin 本地 points2 不过 WiFi），自主巡逻继续冻结。
