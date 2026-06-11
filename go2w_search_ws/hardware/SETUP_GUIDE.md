# Go2W + Jetson NX 硬件集成方案

## 1. 硬件清单

| 组件 | 型号/规格 | 用途 |
|------|----------|------|
| 机器狗 | Unitree Go2W (轮足版) | 移动平台 |
| 车载计算平台 | NVIDIA Jetson Xavier NX (16GB) | 主控，运行 ROS2 + YOLO |
| 存储 | 128GB NVMe M.2 SSD | 系统 + 模型 + 日志 |
| 网络连接 | USB 3.0 转千兆网卡 (RTL8156B) | NX → Go2W 以太网通信 |
| 供电线材 | DC 5.5×2.5mm 公头 + 杜邦线 | Go2W 外部供电口 → NX |
| 固定支架 | 3D打印 NX 底座 (STL见附件) | NX 固定在 Go2W 背部 |
| 散热 | 5V PWM 风扇 + 铝散热片 | NX 主动散热 |
| 可选 | USB 摄像头 (罗技 C920) | 补充视角目标检测 |

## 2. 物理安装

### 2.1 NX 固定

```
Go2W 俯视图 (背部载物区域):

    ┌─────────────────────────────┐
    │          ┌─────────┐        │
    │          │ Jetson   │        │
    │  前方 ←  │   NX     │        │
    │          │ (M3螺丝) │        │
    │          └─────────┘        │
    │    ┌──┐              ┌──┐   │
    │    │ │(USB-Ethernet) │ │    │
    └─────────────────────────────┘
         左后腿          右后腿
```

- Go2W 背部预留 M3 螺丝孔位（间距 65mm × 58mm）
- 3D 打印 NX 底座，通过 M3×6 螺柱连接
- NX 散热鳍片朝上，风扇朝尾部吹风
- 重量重心：NX 约 200g，尽量居中偏后以保持平衡

### 2.2 接线

```
                    ┌──────────────┐
                    │   Go2W 内部   │
                    │  (192.168.   │
                    │  123.161)    │
                    └──────┬───────┘
                           │ 以太网 (Go2W 侧网口)
                    ┌──────┴───────┐
                    │ USB-Ethernet │
                    │   转接器      │
                    └──────┬───────┘
                           │ USB 3.0
                    ┌──────┴───────┐
                    │  Jetson NX   │
                    │ 192.168.     │
                    │ 123.100      │
                    └──────┬───────┘
                           │ DC 供电
                    ┌──────┴───────┐
                    │ Go2W 外部    │
                    │ 供电口 (12V) │
                    └──────────────┘
```

### 2.3 供电方案

| 项目 | 规格 |
|------|------|
| Go2W 外部供电口 | 12V / 5A (DC 5.5×2.5mm) |
| Jetson NX 功耗 | 10W~20W (典型推理负载) |
| 供电方式 | Go2W 外部供电口 → NX DC-in |
| 续航影响 | NX 满载约增加 15-20% 功耗，预计续航减少 10-15 分钟 |

**注意**: Go2W 电池 8000mAh/15Ah，NX 平均功耗 ~15W，整机续航约 60-90 分钟（含 NX）。

## 3. 网络配置

### 3.1 NX 网络设置

```bash
# /etc/netplan/01-go2w-network.yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    # USB-Ethernet 连接 Go2W
    enx001e06300000:  # 替换为实际网卡 MAC
      addresses:
        - 192.168.123.100/24
      routes:
        - to: default
          via: 192.168.123.161
      nameservers:
        addresses: [8.8.8.8]
```

```bash
sudo netplan apply
# 验证
ping 192.168.123.161  # 应该能 ping 通 Go2W
```

### 3.2 Go2W 网络信息

| 项目 | 值 |
|------|------|
| Go2W IP | 192.168.123.161 |
| Go2W DDS Domain | 0 |
| Go2W 相机 RTSP | rtsp://192.168.123.161:8554/camera |
| Go2W SSH | ssh unitree@192.168.123.161 (密码 123) |

## 4. CycloneDDS 配置

ROS2 Galactic 使用 CycloneDDS 与 Go2W 原生 DDS 兼容：

```xml
<!-- ~/.ros/cyclonedds.xml -->
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>enx*</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
```

## 5. Jetson NX 系统安装

### 5.1 系统镜像

```
JetPack 4.6.6 (Ubuntu 20.04 LTS, aarch64)
  - CUDA 11.4
  - TensorRT 8.4+
  - cuDNN 8.3+
```

### 5.2 初始化脚本

见 `setup_jetson.sh`（项目根目录），一键安装 ROS2 Galactic + 开发工具 + 依赖。

## 6. 性能预估

| 指标 | 数值 |
|------|------|
| YOLOv8n TensorRT FP16 推理 | ~25ms/frame (40 FPS) |
| YOLOv8s TensorRT FP16 推理 | ~50ms/frame (20 FPS) |
| Rust 路径规划计算 | <1ms |
| ROS2 节点通信延迟 | <5ms (本机) |
| 搜索速度 (轮式模式) | 1.5-2.5 m/s |
| 10m×10m 区域搜索时间 | 约 3-5 分钟 |
