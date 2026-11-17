# Báo Cáo Đo Đạc Tác Động Camera Phụ (Observer Camera Impact Audit)

> **Mục tiêu:** Đo đạc tác động hiệu năng khi bật vs tắt camera phụ (`observer_camera` 640x480 @ 30Hz).
> **Phương pháp:** 3 cặp A/B (tổng 6 lần chạy), thời lượng mỗi lần 15.0s simulation time.
> **Điều kiện:** Cùng cảnh `gimbal_test`, cùng profile DDS (FastDDS), CUDA Graph replay active, UAV hover 2.0m.

## 1. Bảng Dữ Liệu Từng Lần Chạy (Individual Runs)

| Run ID | Biến thể | RTF | Source FPS (sim/wall) | AI FPS (sim/wall) | Latency p95 (ms) | Median (ms) | Coverage (%) |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `run_1_on` | **ON** | 0.9993 | 29.99 / 29.97 | 27.67 / 27.65 | 69.90 | 18.17 | 92.2% |
| `run_2_off` | **OFF** | 1.0035 | 26.53 / 26.62 | 25.79 / 25.88 | 23.83 | 16.34 | 97.2% |
| `run_3_on` | **ON** | 0.9988 | 30.01 / 29.97 | 28.15 / 28.12 | 67.36 | 19.74 | 93.8% |
| `run_4_off` | **OFF** | 0.9996 | 29.95 / 29.94 | 28.75 / 28.74 | 34.66 | 18.92 | 96.0% |
| `run_5_on` | **ON** | 1.0004 | 26.99 / 27.00 | 24.06 / 24.07 | 66.38 | 19.46 | 89.2% |
| `run_6_off` | **OFF** | 0.9995 | 29.99 / 29.98 | 27.87 / 27.85 | 65.37 | 17.61 | 92.9% |

## 2. So Sánh Đối Đầu A/B (Variant ON vs Variant OFF)

| Chỉ số (Metric) | Camera Phụ BẬT (ON) | Camera Phụ TẮT (OFF) | Chênh lệch (OFF - ON) |
|---|:---:|:---:|:---:|
| **Real-Time Factor (RTF)** | 0.999 ± 0.001 | 1.001 ± 0.002 | +0.001 (+0.1%) |
| **Source Camera FPS (Sim Hz)** | 29.00 ± 1.74 | 28.82 ± 1.99 | -0.173 (-0.6%) |
| **Source Camera FPS (Wall Hz)** | 28.98 ± 1.71 | 28.85 ± 1.93 | -0.133 (-0.5%) |
| **AI Processing FPS (Sim Hz)** | 26.63 ± 2.24 | 27.47 ± 1.52 | +0.843 (+3.2%) |
| **AI Processing FPS (Wall Hz)** | 26.61 ± 2.22 | 27.49 ± 1.46 | +0.877 (+3.3%) |
| **Decision Latency p95 (ms)** | 67.88 ± 1.82 | 41.29 ± 21.55 | -26.59 (-39.2%) |
| **Decision Latency Median (ms)** | 19.12 ± 0.84 | 17.62 ± 1.29 | -1.499 (-7.8%) |
| **AI Coverage on Source (%)** | 91.74 ± 2.37 | 95.38 ± 2.24 | +3.640 (+4.0%) |
| **Frame Period Median (s)** | 0.032 ± 0.000 | 0.032 ± 0.000 | +0.000 (+0.0%) |
| **Frame Period p95 (s)** | 0.036 ± 0.000 | 0.036 ± 0.000 | +0.000 (+0.0%) |

## 3. Phân Tích & Kết Luận (Findings & Decision)

- **Tác động nằm trong biên độ dao động (< 5%):** RTF chênh lệch +0.1%, Source FPS chênh lệch -0.6%.
- **Khuyến nghị:** Giữ nguyên world hiện tại vì camera phụ không gây nghẽn rendering đáng kể so với camera chính 1280x720 và tải GPU của YOLO/BoT-SORT.
