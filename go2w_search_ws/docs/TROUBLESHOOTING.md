# 改造过程问题记录与解决方法

> 按时间顺序记录改造过程中遇到的实际问题、现象、根因、解决方法。
> 每个问题都基于实测,非推测。

---

## 问题 1:载荷NX 连不到狗主控(网线插着但ping不通)

### 现象
- 载荷NX 网线插着连狗,`enP8p1s0` 物理有连接(carrier=1)
- ping 狗主控 `192.168.123.161` → 100% 丢包
- 扫 123 网段 → 0 个设备
- 在 144 网段找到一个设备 `192.168.144.108`(响应ICMP、开telnet23),但**不发DDS、不开SSH、不是狗主控**

### 错误的中间判断(走了弯路)
一度判断"网线插错口,连到了狗的MCU而非主控",建议去现场重新插线。

### 真正根因
**载荷NX 有两个有物理连接的网口,而狗主控连在第二个(USB转网口)上,但那个口没配IP:**

| 网口 | 类型 | IP | 连的设备 |
|---|---|---|---|
| `enP8p1s0` | 板载网口 | 192.168.144.36 | 狗MCU(144.108) |
| `enxc8a362616c4c` | **USB转网口(AX88179)** | **无IP** | **狗主控(123.161)** ← 真正连狗的口 |

之前一直盯着板载网口 `enP8p1s0` 查,完全忽略了 USB 转网口也有物理连接。是用户提示"NX用的是USB转网口"才注意到第二个网卡。

### 解决方法
给 USB 转网口配 123 网段静态IP(用 nmcli 持久化):
```bash
nmcli connection add type ethernet ifname enxc8a362616c4c con-name go2-dog
nmcli connection modify go2-dog ipv4.method manual ipv4.addresses 192.168.123.100/24
nmcli connection modify go2-dog ipv4.gateway ""           # 关键:不设网关
nmcli connection modify go2-dog ipv4.route-metric 200     # 低优先级,不抢热点默认路由
nmcli connection modify go2-dog connection.autoconnect yes
nmcli connection up go2-dog
```
**关键点:** 不设网关 + route-metric 200,确保狗链路不抢走 WiFi 的默认路由(否则NX会断网)。

### 教训
- `ip link show` 看所有网卡,不要只盯一个;USB转网口命名 `enx<MAC>` 是典型特征
- ping不通时,先确认"目标设备在哪个二层广播域",用 `arp-scan --localnet` 扫真实链路上的设备

---

## 问题 2:PC ↔ 载荷NX 的 ROS2 DDS 单向数据故障

### 现象
- PC(Galactic)和 NX(Humble)互相能 `ros2 topic list` 看到对方话题(发现OK)
- **但 PC 收不到 NX 发的数据;NX 反向能收到 PC 的数据**(单向)

### 根因
**Galactic(EOL)和 Humble 的默认 RMW(FastDDS)在类型哈希/QoS 上有细微差异**,导致:
- 发现协议(SDPP)跨版本能看到(所以 topic list 有)
- 但 RTPS DataWriter/DataReader 配对需要类型哈希完全匹配,老 Galactic 读不动新 Humble 的序列化
- 反向能通是因为新版本(Humble)对老版本兼容性更好

### 解决方法
**PC 从 Galactic 升级到 Humble,和 NX 版本对齐。**

但 PC 是 Ubuntu 20.04,Humble 没有官方 focal 包(只支持 22.04)。最终方案:**PC 用 Docker 跑 Humble**(`--net=host` 让 DDS 走主机网络),对现有 Galactic/noetic/conda 项目零侵入。

验证:容器(Humble)↔ NX(Humble)双向心跳,数据全通。

### 教训
- ROS2 跨机通信,**两端 RMW 版本必须一致**;跨大版本(Galactic↔Humble)不要指望"能发现就能通信"
- "能 list 到 topic 但 echo 收不到数据" = 典型的类型/QoS 不匹配

---

## 问题 3:`cyclonedds==0.10.2` 在 Jetson(arm64)上 pip 安装失败

### 现象
```
pip install cyclonedds==0.10.2
→ error: Getting requirements to build wheel did not run successfully
→ Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```

### 根因
`cyclonedds` Python 包在 arm64 上**没有预编译 wheel**,pip 要从源码 build,而 build 要求**先有一个编译好的 CycloneDDS C 库**(`CYCLONEDDS_HOME`)。直接 pip 装不行。

### 解决方法
**手动两步装**(宇树官方文档流程):
```bash
# 1. 编译 C 库
git clone -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/CycloneDDS -DBUILD_TESTING=OFF ..
cmake --build . --target install -j$(nproc)

# 2. 装 python 绑定(指向刚编译的C库)
export CYCLONEDDS_HOME=$HOME/CycloneDDS
pip install --user cyclonedds==0.10.2
```

### 注意
- 运行时也要 `export LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$LD_LIBRARY_PATH`,否则 `import cyclonedds` 找不到 `libddsc.so`

---

## 问题 4:`unitree_sdk2py` pip 找不到 + github clone 超时

### 现象1:pip 找不到
```
pip install unitree_sdk2py
→ ERROR: No matching distribution found for unitree_sdk2py
```
**根因:** `unitree_sdk2py` 不在 PyPI,只能从 github 源码装。

### 现象2:github clone 超时/失败
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
→ error: RPC failed; curl 16 Error in the HTTP2 framing layer
→ fatal: Could not resolve host: github.com   (间歇性)
```

### 根因
- 华为热点 + USB网口网络抖动,HTTPS 大对象(pack数据)传输中途断流
- git 默认用 HTTP/2,对丢包敏感
- DNS 也间歇性失效(但只是瞬时,重测又好)

### 解决方法
**强制 git 用 HTTP/1.1 + 增大 buffer**:
```bash
git -c http.version=HTTP/1.1 \
    -c http.postBuffer=524288000 \
    -c http.lowSpeedLimit=0 \
    -c http.lowSpeedTime=999999 \
    clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
```
**备选方案:** PC 网络稳定时,在 PC clone 好,scp 传到 NX。

### 教训
- 不稳定网络下 clone github,优先 `--depth 1` + 强制 HTTP/1.1
- `git ls-remote` 能通但 `git clone` 失败 = 数据传输阶段断流,不是认证/网络不通

---

## 问题 5:`pip install unitree_sdk2py` 超时(卡在下载依赖)

### 现象
源码目录 `pip install --user .` 超时(3分钟未完成)。

### 根因
`setup.py` 的 `install_requires` 有 `opencv-python`,pip 去网络下载时卡住(网络抖动)。

### 解决方法
**离线装,跳过依赖**(`--no-deps`):
```bash
pip install --user --no-deps --no-build-isolation .
```
cyclonedds 已装、numpy NX 自带、opencv 验证期用不上,所以跳过依赖不影响功能。

---

## 问题 6:误判华为热点有 AP 隔离

### 现象
初期 ping 扫描 192.168.43.0/24,除了网关和本机几乎无应答;某个IP短暂 REACHABLE 后立刻 100% 丢包。

### 错误判断
一度怀疑华为热点开了 AP 隔离(客户端隔离),禁止设备间互访。

### 真相
拿到 NX 的真实 IP(192.168.43.41)后直接 ping,**完全通,0%丢包**。之前扫不到是因为:
- ping 扫描时 NX 可能休眠/省电
- 并行 ping 254 个地址本身有丢包干扰

### 教训
- 不要凭"ping扫描扫不到"就断定隔离;**直接 ping 已知目标IP** 才是准的
- AP 隔离的判断要用"两个已知在线设备互ping",不是扫全网段

---

## 问题 7:MID360(USB版)识别失败 — USB供电不足

### 现象
- MID360 是 **USB 接口版**(非标准网口版),插在载荷NX的USB上
- `lsusb` 里**没有任何 Livox/雷达设备**
- 链路上抓不到雷达广播包
- 网口扫描 192.168.1.x 全空

### 根因(dmesg 决定性证据)
USB 端口 `1-2.4`(USB 2.0 Hub 第4口)上的设备**反复枚举失败**:
```
usb 1-2.4: new low-speed USB device number 14
usb 1-2.4: device descriptor read/64, error -32       ← EPIPE 端点停滞
usb 1-2.4: device descriptor read/64, error -32       ← 反复出错
usb 1-2-port4: attempt power cycle                     ← 内核尝试重新供电
usb 1-2.4: Device not responding to setup address.
usb 1-2-port4: unable to enumerate USB device          ← 最终失败
```
**`error -32`(EPIPE)+ `attempt power cycle` = USB Hub 供电不足的典型症状。** MID360被识别成 `low-speed`(低速)也是供电不足导致无法完成正常握手。

根因排序:
1. ⭐⭐⭐ **USB Hub 供电不足** — MID360/相机功耗大,USB 2.0 Hub 无源端口供电不够
2. ⭐⭐ USB 线缆质量差/接触不良
3. ⭐ 设备本身故障

### 当前状态
- ✅ livox-sdk2 已编译安装(`/usr/local/lib/liblivox_lidar_sdk_shared.so`)
- ✅ 标准 mid360 配置文件在(`~/ThirdPartyLib/livox-sdk2/samples/.../mid360_config.json`)
- ❌ **雷达硬件层面没被系统识别,装驱动也没用**

### 解决方向(需现场操作)
1. **换带独立电源的 USB Hub**(或直接插NX主板USB口 + 给设备独立供电)
2. 检查 MID360 是否需要独立电源适配器(USB线可能只传数据)
3. 换一根USB线排除线缆问题
4. 确认插在 USB 3.0 口(蓝色)而非 USB 2.0 口

### 教训
- 设备 `lsusb` 看不到时,先看 `dmesg` 有没有枚举失败记录 — 能区分"没插"vs"插了但识别失败"
- `error -32` + `power cycle` 几乎可锁定供电问题

---

## 附录:载荷NX 环境清单(实测)

| 项 | 状态 | 位置/版本 |
|---|---|---|
| 系统 | Ubuntu 22.04.5 + L4T 36.4.4 | Jetson Orin NX 16GB (p3767-0000) |
| 功率模式 | 25W(模式3,默认) | 后期跑SLAM/检测需切更高 |
| ROS2 Humble | ✅ | /opt/ros/humble |
| mavros | ✅ | 2.14.0(PX4接入栈) |
| CycloneDDS C库 | ✅ | ~/CycloneDDS (libddsc.so 0.10.2) |
| cyclonedds python | ✅ | ~/.local (0.10.2) |
| unitree_sdk2py | ✅ | ~/unitree_sdk2_python (1.0.1) |
| livox-sdk2 | ✅(已存在) | ~/ThirdPartyLib/livox-sdk2 (MID360用) |
| MID360 驱动 | ❓ 待查 | livox-sdk2在但驱动节点没确认 |
| 云台相机 | ❓ 待查 | 无 /dev/video* |
| PX4 | ❌ 未接 | 无 /dev/ttyACM* |
