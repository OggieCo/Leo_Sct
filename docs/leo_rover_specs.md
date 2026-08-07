# Leo Rover — Hardware Specification

Leo Rover is a compact, 4-wheeled, remote-controlled robot designed for indoor
and outdoor robotic project development, learning, and research applications. It
is equipped with a built-in computer, a high-resolution camera, and a powerful
battery, making it suitable for various tasks, including autonomous navigation,
obstacle detection, and remote monitoring.

## Main parameters

| Parameter | Value |
|---|---|
| Dimensions (LxWxH) | 424 mm x 445 mm x 303 mm |
| Weight | ≈ 7 kg |
| Maximum payload | ≈ 5 kg* |
| Maximum linear speed | ≈ 0.4 m/s |
| Maximum angular speed | ≈ 1 Rad/s |
| Estimated max. obstacle size | ≈ 70 mm |
| IP protection rating | IP 55 |
| Operating temperature | -10 °C to +40 °C |
| Run time | Up to 4 hours with standard battery |
| Connection range | Up to 100 m |

\* on standard tires

### Traction parameters

| Parameter | Value |
|---|---|
| Track Width | 354 mm |
| Wheelbase length | 295 mm |
| Ground clearance | 108 mm |
| Climb grade (no payload) | 45° (100 %) |
| Climb grade with 5kg payload | 45° (100 %) |
| Hill grade traversal | 45° (100 %) |
| Nominal torque | 4 Nm |
| Maximum torque | 5.6 Nm |

### Rover overview

![Overview of main features of Leo Rover](https://docs.fictionlab.pl/img/robots/leo/specification/leo-1.9-overview-light.webp)

## Hardware specification

### Dimensions

![Leo Rover Dimensions](https://docs.fictionlab.pl/img/robots/leo/specification/leo-1.9_dimensions-light.webp)

### Components

| Name | Quantity | Description |
|---|---|---|
| Built-in computer | 1 | **Raspberry Pi 5** — single-board computer developed by Raspberry Pi Ltd. Key features: Processor: Broadcom BCM2712, 2.4GHz quad-core 64-bit Arm Cortex-A76 CPU with cryptography extensions. Memory: 4GB LPDDR4X-4267 SDRAM. Storage: Micro SD card slot with support for high-speed SDR104 mode; PCIe 2.0 x1 interface for fast peripherals (requires separate M.2 HAT or adapter). Connectivity: Gigabit Ethernet (supports PoE+ with separate PoE+ HAT); Dual-band 802.11ac Wi-Fi; Bluetooth 5.0 / Bluetooth Low Energy (BLE); 2 x USB 3.0 ports supporting simultaneous 5Gbps operation; 2 x USB 2.0 ports; 2 x micro-HDMI ports supporting resolutions up to 4kp60 with HDR support; 2 x 4-lane MIPI camera/display transceivers; standard 40-pin GPIO header |
| Wi-Fi adapter | 1 | Alfa AWUS036ACS: USB 2.0 Wi-Fi adapter (Realtek RTL8811AU). Dual-band 802.11ac (AC600: 150 Mbps 2.4GHz + 433 Mbps 5GHz) |
| Antenna | 1 | Dual-band (2.4 GHz / 5 GHz) placed on the top of the robot |
| Front camera | 1 | Arducam 12.3MP 477M HQ Camera Module for Raspberry Pi with 158°(D) M12 Wide Angle Lens |
| DC motors | 4 | Bühler Motors 1.61.077.414 connected to LeoCore |
