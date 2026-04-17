# sdhci-cv1800

[![Rust](https://img.shields.io/badge/rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

CVI SoC (CV1800/SG2002) SDHCI 控制器驱动，用于 SDIO WiFi 设备（如 AIC8800）。

## 概述

本 crate 实现了 Sophgo CV1800/SG2002 系列 SoC 的 SDHCI (Secure Digital Host Controller Interface) 控制器驱动，支持 SDIO 协议的 WiFi 设备通信。

### 主要特性

- ✅ **SDIO 协议支持**：完整的 CMD52/CMD53 命令实现
- ✅ **SDIO 卡枚举**：自动完成 CMD5 → CMD3 → CMD7 初始化流程
- ✅ **PIO 数据传输**：基于程序控制 I/O 的数据读写
- ✅ **中断驱动**：支持中断模式的高效数据传输
- ✅ **动态时钟配置**：支持 400KHz ~ 50MHz 的时钟切换
- ✅ **4-bit 总线**：支持高速 4-bit 总线模式
- ✅ **SoC 硬件初始化**：完整的 Pinmux、时钟、复位配置
- ✅ **精准延时**：基于 RISC-V time CSR 的微秒级延时

## 硬件支持

### 支持的 SoC

- ✅ Sophgo **SG2002**
- ✅ Sophgo **CV1800**
- ✅ Sophgo **CV1810**
- ✅ Sophgo **CV1811**

### 支持的 WiFi 设备

- ✅ AIC8800 (SDIO WiFi)
- ✅ 其他符合 SDIO 3.0 规范的设备

## 架构设计

### 模块结构

```
sdhci-cv1800/
├── src/
│   ├── lib.rs        # 主驱动实现，实现 SdioHost trait
│   ├── hw_init.rs    # SoC 级硬件初始化（时钟/复位/Pinmux）
│   ├── irq.rs        # 中断处理和状态管理
│   └── regs.rs       # SDHCI 寄存器定义
├── Cargo.toml
└── README.md
```

### 驱动层次

```
┌─────────────────────────────────────┐
│   WiFi 驱动 (aic8800_fdrv)          │
├─────────────────────────────────────┤
│   SdioHost Trait (axdriver_sdio)    │
├─────────────────────────────────────┤
│   sdhci-cv1800 (本驱动)             │
│   ├── SDHCI 控制器操作               │
│   ├── SDIO 协议 (CMD52/CMD53)       │
│   ├── 中断管理                       │
│   └── SoC 硬件初始化                 │
├─────────────────────────────────────┤
│   硬件层 (SG2002 SDMMC 控制器)       │
└─────────────────────────────────────┘
```

### 设计特点

1. **初始化阶段**：使用轮询模式等待硬件就绪
2. **运行阶段**：切换到中断驱动模式，提高效率
3. **异步支持**：通过 `poll_fn` 支持异步等待原语

## 使用方法

### 基本用法

```rust
use sdhci_cvh1800::CviSdhci;
use axdriver_sdio::SdioHost;

// 1. 创建 SDHCI 控制器实例
// SDIO1 基地址: 0x05000000 (SG2002)
let mut sdhci = CviSdhci::new(0x0500_0000);

// 2. 初始化 SDIO 卡
// 自动完成: CMD5 → CMD3 → CMD7 → 高速模式 → 4-bit 总线
sdhci.init()?;

// 3. 读取 vendor/device ID
let (vendor, device) = sdhci.vendor_device_id();
println!("SDIO Card: vendor=0x{:04x}, device=0x{:04x}", vendor, device);

// 4. 读写寄存器
// 读取 Function 1 的寄存器 0x00
let value = sdhci.read_byte(1, 0x00)?;

// 写入 Function 1 的寄存器 0x04
sdhci.write_byte(1, 0x04, 0xFF)?;

// 5. FIFO 数据传输
let mut buffer = [0u8; 512];
// 从 Function 1 的地址 0x0 读取 512 字节
sdhci.read_fifo(1, 0x0, &mut buffer)?;

// 向 Function 1 的地址 0x0 写入 512 字节
sdhci.write_fifo(1, 0x0, &buffer)?;
```

### SoC 硬件初始化

如果需要完整的 SoC 级初始化（包括 Pinmux、时钟、复位）：

```rust
use sdhci_cvh1800::hw_init::{Sdio1HwConfig, sdio1_hw_init};

// 配置 SoC 硬件参数
let hw_config = Sdio1HwConfig {
    crg_base_va: 0x0300_2000,          // CRG 基地址
    sysctrl_base_va: 0x0300_0000,       // System Control 基地址
    rtcsys_ctrl_base_va: 0x0502_5000,   // RTC 控制器基地址
    rtcsys_io_base_va: 0x0502_7000,     // RTC IO 基地址
    sdio1_base_va: 0x0500_0000,         // SDIO1 控制器基地址
    delay_us: |us| {
        // 提供微秒级延时函数
        // 使用 RISC-V time CSR 实现
        delay_us_impl(us);
    },
};

// 执行 SoC 硬件初始化
// 步骤: Pinmux → 时钟使能 → 复位解除 → 卡检测覆写
sdio1_hw_init(&hw_config);
```

### 中断配置

```rust
use sdhci_cvh1800::irq;

// 注册中断处理程序
// SDIO1 中断号: 38 (SG2002)
let irq_num = 38;
axplat::irq::register(irq_num, sdhci_irq_handler)?;

// 在数据传输期间屏蔽卡中断
sdhci.mask_card_irq();
// ... 执行 CMD52/CMD53 ...
sdhci.unmask_card_irq();
```

## 时钟配置

### 支持的时钟频率

| 模式 | 时钟频率 | 用途 |
|------|----------|------|
| 初始化 | 400 KHz | SDIO 卡枚举阶段 |
| 默认 | 25 MHz | 标准速度模式 |
| 高速 | 50 MHz | 高速模式 (High-Speed) |

### 时钟切换

驱动会自动根据以下情况切换时钟：

1. **初始化阶段**：自动使用 400KHz
2. **检测到高速支持**：自动切换到 50MHz
3. **不支持高速**：回退到 25MHz

## 内存映射

### SG2002 SDIO1 寄存器地址

| 模块 | 基地址 | 说明 |
|------|--------|------|
| SDIO1 控制器 | 0x0500_0000 | SDHCI 寄存器 |
| CRG | 0x0300_2000 | 时钟/复位发生器 |
| TOP_MISC | 0x0300_0000 | 系统控制寄存器 |
| RTCSYS_CTRL | 0x0502_5000 | RTC 域控制寄存器 |
| RTCSYS_IO | 0x0502_7000 | RTC 域 IO 复用寄存器 |

### SDHCI 关键寄存器偏移

| 寄存器 | 偏移 | 说明 |
|--------|------|------|
| SDHCI_ARGUMENT | 0x08 | 命令参数 |
| SDHCI_COMMAND | 0x0E | 命令寄存器 |
| SDHCI_RESPONSE | 0x10 | 响应寄存器 |
| SDHCI_BUFFER | 0x20 | 数据缓冲端口 |
| SDHCI_PRESENT_STATE | 0x24 | 当前状态寄存器 |
| SDHCI_CLOCK_CONTROL | 0x2C | 时钟控制寄存器 |
| SDHCI_CAPABILITIES | 0x40 | 能力寄存器 |

## 性能特性

### 数据传输模式

- **PIO 模式**：当前实现，CPU 直接参与数据传输
- **DMA 模式**：预留接口（`cmd53_read`/`cmd53_write`），待未来扩展

### 中断响应

- **轮询模式**：初始化阶段使用
- **中断模式**：运行时使用，零 CPU 自旋
- **中断延迟**：< 10μs（典型值）

## 依赖项

```toml
[dependencies]
axdriver_sdio = { git = "https://github.com/Ticonderoga2017/axdriver_sdio.git" }
axtask = { git = "https://github.com/Ticonderoga2017/sg2002-arceos.git", features = ["multitask"] }
axpoll = "0.1"
log = "0.4"
```

## 构建要求

- **Rust**：1.70 或更高版本
- **目标架构**：riscv64gc-unknown-none-elf
- **工具链**：nightly-riscv64gc-unknown-none-elf

### 构建命令

```bash
# 构建 debug 版本
cargo build

# 构建 release 版本
cargo build --release

# 查看生成的二进制大小
ls -lh target/riscv64gc-unknown-none-elf/debug/libsdhci_cvh1800.rlib
```

## 配置选项

### axconfig.toml 配置

确保在 `axplat-riscv64-sg2002/axconfig.toml` 中正确配置：

```toml
[devices]
# 定时器频率 (用于 delay_us)
timer-frequency = 4_000_000  # 4MHz

# SDIO1 基地址
sdio1-paddr = 0x05000000    # SDIO1 物理基址
sdio1-irq = 38               # SDIO1 中断号

# SoC 子系统基址 (用于硬件初始化)
crg-paddr = 0x03002000           # CRG 基地址
sysctrl-paddr = 0x03000000       # System Control 基地址
rtcsys-ctrl-paddr = 0x05025000   # RTC 控制器基地址
rtcsys-io-paddr = 0x05027000     # RTC IO 基地址
```

## 故障排查

### 编译错误

**问题**：`error: linking with 'cc' failed`
```
**解决**：确保使用正确的 RISC-V 工具链
```bash
rustup target add riscv64gc-unknown-none-elf
export RUSTFLAGS="-C linker=riscv64-unknown-elf-gcc"
```

### 运行时错误

**问题**：SDIO 卡初始化超时
```
**排查步骤**：
1. 检查硬件连接（SDIO 卡是否正确插入）
2. 验证时钟配置（CRG 寄存器是否正确配置）
3. 检查中断配置（中断号是否正确）
4. 使用 `hw_init::sdio1_hw_dump()` 查看寄存器状态
```

**问题**：数据传输 CRC 错误
```
**排查步骤**：
1. 降低时钟频率（从 50MHz 降到 25MHz）
2. 检查信号完整性（示波器查看 CLK/CMD/DAT 信号）
3. 验证 4-bit 总线配置
4. 尝试使用 1-bit 总线模式
```

## 参考文档

### 规范文档

- [SDIO 3.0 规范](https://www.sdcard.org/downloads/pls/simplified_specs/part1_300.pdf)
- [SD Host Controller 规范 v3.0](https://www.sdcard.org/downloads/pls/simplified_specs/partA2_300.pdf)

### 硬件文档

- [SG2002 技术参考手册](https://github.com/sophgo/sophgo-doc)
- [CV1800/SG2002 数据手册](https://github.com/sophgo/sophgo-doc)

### 相关项目

- [axdriver_sdio](https://github.com/Ticonderoga2017/axdriver_sdio) - SDIO 抽象层
- [sg2002-arceos](https://github.com/Ticonderoga2017/sg2002-arceos) - ArceOS 操作系统
- [aic8800_fdrv](https://github.com/Ticonderoga2017/aic8800_fdrv) - AIC8800 WiFi 驱动

## 许可证

本项目采用 [Apache License 2.0](LICENSE) 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！

### 贡献指南

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 作者

Ticonderoga2017

## 致谢

- Sophgo 团队提供 SG2002 硬件文档
- ArceOS 社区提供操作系统框架
- RISC-V 社区提供工具链支持

## 更新日志

### v0.1.0 (2024-04-17)

- ✅ 初始版本发布
- ✅ 实现基本的 CMD52/CMD53 命令
- ✅ 支持 SDIO 卡枚举和初始化
- ✅ 实现 PIO 数据传输
- ✅ 支持中断驱动模式
- ✅ 实现 SoC 硬件初始化
- ✅ 实现精准延时功能

### TODO

- [ ] 支持 DMA 模式传输
- [ ] 支持更多 SDIO 功能（Power Management 等）
- [ ] 添加更多调试和诊断功能
- [ ] 优化性能（减少 CPU 占用）
- [ ] 添加更完整的错误处理和恢复机制
