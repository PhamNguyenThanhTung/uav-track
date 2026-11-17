# Báo Cáo Audit Cấu Trúc Dự Án (Historical Project Structure Snapshot)

> Historical snapshot from before the BoT-SORT migration. Its runtime map, inventory, and test counts describe the pre-migration tree; use `PROGRESS.md` and `docs/tracking/botsort_migration_report.md` for current state. Preserved because earlier review notes cite this audit.

**Ngày thực hiện**: 2026-10-14  
**Phạm vi**: `/home/tungt/UAV_Project/project`  
**Nguyên tắc**: Chỉ audit, đọc và phân loại dựa trên reference thực tế trong mã nguồn. **Không xóa, không di chuyển, không đổi tên, không chỉnh sửa mã nguồn trong task này.**

---

## 1. Cấu Trúc Hiện Tại Của Dự Án (Current Tree Rút Gọn)

```text
/home/tungt/UAV_Project/project/
├── AGENTS.md                                   # Quy tắc vận hành và ranh giới làm việc cho AI agents
├── PROGRESS.md                                 # Trạng thái kỹ thuật hiện tại và baseline đã nghiệm thu
├── README.md                                   # Tài liệu hướng dẫn sử dụng và kiến trúc tổng quan
├── .gitignore                                  # Cấu hình bỏ qua build, log, venv, runs, weights
├── benchmark_evaluator.py                      # (Symlink) -> evaluation/benchmark_evaluator.py
├── metric_calculator.py                        # (Symlink) -> evaluation/metric_calculator.py
├── launch_simulation.sh                        # Launcher tổng hợp mô phỏng (Gazebo + PX4 + Bridge + DDS)
├── launch_phase4a_review.sh                    # Launcher chạy review Phase 4A chia tab terminal
│
├── config/                                     # Cấu hình runtime và simulation
│   └── sim/
│       ├── gazebo_rendering.sh                 # Thiết lập GPU Gallium D3D12 WSL2 cho Gazebo
│       └── ros_gz_bridge.yaml                  # Cấu hình bridge ROS 2 <-> Gazebo Harmonic
│
├── docs/                                       # Tài liệu đặc tả và ghi chú kỹ thuật
│   ├── project_overview.md                     # Tổng quan kiến trúc hệ thống 5 giai đoạn
│   ├── system_contract.md                      # Hợp đồng giao tiếp ROS 2 (topics, QoS, camera axis)
│   ├── tracking/
│   │   └── botsort_migration_spec.md           # Đặc tả kỹ thuật di chuyển sang BoT-SORT + GMC + ReID
│   ├── simulation/                             # Báo cáo kết quả các mốc mô phỏng lịch sử (Phase 1, 2, 3)
│   │   ├── gimbal_camera_phase1.md
│   │   ├── phase2_altitude_control.md
│   │   └── phase3_integration.md
│   ├── notes/                                  # 18 tài liệu ghi chú phân tích sâu và audit quá khứ
│   │   ├── phase4a_audit_2026-10-14.md
│   │   ├── deepsort_uav_deep_research_2026-10-12.md
│   │   ├── uav_tracking_landscape_2026-10-13.md
│   │   ├── uav_tracking_multiplatform_2026-10-13.md
│   │   ├── phase4a_final_review_2026-10-13.md
│   │   ├── phase4a_final_review_evidence_2026-10-13.md
│   │   ├── phase4a_motion_extent_spec_2026-10-13.md
│   │   ├── phase4a_review_2026-10-13.md
│   │   ├── tracking_identity_2026-10-09.md
│   │   ├── tracking_phase4a_completion_2026-10-09.md
│   │   ├── project_audit_2026-10-06.md
│   │   ├── project_audit_followup_2026-10-06.md
│   │   ├── project_audit_2026-10-13.md
│   │   ├── camera_clearance_2026-10-07.md
│   │   ├── gazebo_fps_diagnosis_2026-10-07.md
│   │   ├── gazebo_gpu_2026-10-06.md
│   │   ├── upright_fullbody_2026-10-07.md
│   │   └── wsl_clock_fix_2026-10-07.md
│   ├── research/                               # (Thư mục trống từ Sep 23)
│   └── setup/                                  # (Thư mục trống từ Sep 23)
│
├── evaluation/                                 # Hệ thống đánh giá, benchmark và dữ liệu ground truth
│   ├── benchmark_evaluator.py                  # Node đánh giá benchmark khớp frame thời gian thực
│   ├── metric_calculator.py                    # Thư viện tính toán chỉ số bám mục tiêu và sai số
│   ├── frame_matcher.py                        # Buffer trượt khớp frame camera với detection/telemetry
│   ├── evaluate_phase4a_target_guard.py        # Đánh giá cổng nghiệm thu Phase 4A với ground truth
│   ├── profile_phase4a_target_guard.py         # Profiler đo đạc thời gian chạy GPU/CPU từng module
│   ├── replay_phase4a_target_guard.py          # Replay hồi quy nhanh TargetIdentityManager từ cache NPZ
│   ├── annotate_phase4a_target_events.py       # Tool tạo nhãn ground truth từ video raw
│   ├── check_tracking_reentry.py               # Kiểm tra trường hợp mục tiêu rời khỏi và tái nhập FOV
│   ├── render_identity_comparison.py           # Render video so sánh giữa DeepSORT thô và Guard
│   ├── audit_camera_fps_accounting.py          # Script đo đạc kiểm toán FPS và RTF camera Gazebo
│   ├── fps_frame_accounting_audit_report.md    # Báo cáo kết quả kiểm toán FPS camera
│   ├── tracking_3_people_test.py               # Observer ghi dữ liệu kịch bản 3 người cắt nhau
│   ├── audit_tracking_identity.py              # Script audit dữ liệu danh tính từ file observations.npz
│   ├── tracking_loss_intervals.py              # Helper đóng khoảng thời gian mất dấu
│   ├── tracking_identity_candidate.py          # [Prototype cũ] Thử nghiệm subclassing DeepSORT
│   ├── tracking_appearance.py                  # [Prototype cũ] Helper HSV torso ban đầu
│   ├── replay_tracking_identity.py             # [Prototype cũ] Script replay cho prototype cũ
│   ├── test_*.py                               # 7 file unit tests (đều PASS 100%)
│   ├── annotations/                            # Dữ liệu gán nhãn thủ công (CSV, JSON, contact sheets)
│   └── results/                                # Kết quả benchmark baseline và 58 thư mục chạy (runs/)
│
├── simulation/                                 # Tài nguyên mô phỏng Gazebo Harmonic & PX4
│   ├── models/                                 # Custom models (custom_x500_gimbal_down, targets)
│   ├── worlds/                                 # Worlds SDF (gimbal_test, tracking_3_people_test, v.v.)
│   ├── run_sim_phase1.sh                       # Script chạy Gazebo Phase 1 đơn lẻ
│   ├── run_sim_phase3.sh                       # Script chạy Gazebo Phase 3 tích hợp PX4 plugins
│   └── run_tracking_3_people_test.sh           # Script tự động hóa chạy benchmark 3 người
│
├── skills/                                     # Các quy trình thao tác chuẩn (SOP) cho Agents
│   ├── bug-fix/SKILL.md
│   ├── delegation/SKILL.md
│   ├── phase-review/SKILL.md
│   └── runtime-verification/SKILL.md
│
├── src/                                        # ROS 2 packages nguồn chính
│   ├── uav_control/                            # Package C++ điều khiển bay Offboard
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── src/altitude_control.cpp            # Node C++ offboard takeoff và giữ tọa độ horizontal
│   │   └── test/test_altitude_control.cpp      # GTest unit test suite (7 tests)
│   └── uav_perception/                         # Package Python nhận diện và bám mục tiêu
│       ├── package.xml
│       ├── setup.py, setup.cfg
│       ├── README.md
│       ├── resource/uav_perception
│       ├── uav_perception/
│       │   ├── __init__.py
│       │   ├── yolo_detector.py                # Node chính: YOLO + DeepSORT + Guard + Gimbal
│       │   ├── target_identity.py              # FSM quản lý danh tính, gating 7 cổng, extent guard
│       │   ├── gimbal_controller.py            # Bộ điều khiển visual servoing 2-DOF (yaw, pitch)
│       │   ├── image_utils.py                  # Giải mã ảnh ROS, padding step, HSV torso descriptor
│       │   ├── detection_utils.py              # Định dạng thông điệp Detection2DArray
│       │   └── telemetry_utils.py              # Mã hóa/giải mã Float64MultiArray telemetry
│       └── test/                               # 7 file pytest unit tests (67 tests, PASS 100%)
│
└── .ua/                                        # Dữ liệu sinh tự động từ công cụ Understand-Anything
```

---

## 2. Bản Đồ Kiến Trúc Runtime Hiện Tại (Runtime Map)

Bảng phân rã 10 thành phần chức năng theo đúng mã nguồn đang thực thi:

| STT | Thành phần (Component) | File / Node chịu trách nhiệm | Input chính | Output chính | Có trùng lặp (Duplicate) logic không? |
|---|---|---|---|---|---|
| 1 | **Camera Input** | `simulation/models/custom_x500_gimbal_down/model.sdf`<br>`config/sim/ros_gz_bridge.yaml`<br>`src/uav_perception/uav_perception/yolo_detector.py` | Sensor Gazebo Harmonic (`1280x720@30Hz`) | Topic ROS 2: `/custom_x500/camera/image_raw` (`sensor_msgs/Image`) | `benchmark_evaluator.py` và `audit_camera_fps_accounting.py` cũng subscribe độc lập để đo lường. |
| 2 | **Person Detection** | `src/uav_perception/uav_perception/yolo_detector.py` (`YOLODetectorNode`) | Ảnh BGR numpy từ `image_utils.image_to_bgr` | Bounding boxes người `deepsort_dets` + Topic `/yolo/raw_detections` | `tracking_3_people_test.py` và `profile_phase4a_target_guard.py` có code chạy YOLO tương tự để ghi/đo dữ liệu. |
| 3 | **Tracking / ID Assignment** | `src/uav_perception/uav_perception/yolo_detector.py` (sử dụng thư viện `deep_sort_realtime`) | Detections từ YOLO + Embeddings | Track objects (`track_id`, bbox LTRB gốc) | `evaluation/tracking_identity_candidate.py` chứa prototype kế thừa DeepSORT cũ; `tracking_3_people_test.py` cũng gọi DeepSORT. |
| 4 | **Re-ID** | 1. Neural: MobileNet trong DeepSORT (`yolo_detector.py:336`)<br>2. Color Torso: `src/uav_perception/uav_perception/image_utils.py` (`torso_histogram`) | Crop ảnh người từ DeepSORT | Vector 128-D (DeepSORT) và Vector 192-D (HSV Torso) | `evaluation/tracking_appearance.py` là bản nháp ban đầu của `torso_histogram`, hiện đã được chuẩn hóa vào `image_utils.py`. |
| 5 | **Camera Motion Compensation (GMC)** | **CHƯA CÓ IMPLEMENTATION TRONG MÃ NGUỒN RUNTIME** | N/A | N/A | Chưa có mã nguồn chạy runtime. Đã có tài liệu đặc tả `docs/tracking/botsort_migration_spec.md` chuẩn bị tích hợp ORB GMC. |
| 6 | **Target Selection** | `src/uav_perception/uav_perception/target_identity.py` (`TargetIdentityManager.update`) | Fresh tracks từ DeepSORT | Khóa target ID `1` từ quan sát sạch đầu tiên (`_anchor_feature`, `_auxiliary_anchor`) | Duy nhất tại `target_identity.py`. |
| 7 | **Target Guard / Lost / Reacquisition** | `src/uav_perception/uav_perception/target_identity.py` (`TargetIdentityManager`) | Fresh tracks, timestamp nguồn, detection bboxes | Trạng thái FSM (`TRACKING`, `UNCERTAIN`, `LOST`), bbox xác thực, topic `/yolo/detections` | Duy nhất trong production. Bản cũ ở `evaluation/tracking_identity_candidate.py` không còn được runtime sử dụng. |
| 8 | **Gimbal Visual Servoing** | `src/uav_perception/uav_perception/gimbal_controller.py` (`GimbalController`) | Bounding box mục tiêu đã xác thực `tracked_bbox` | Lệnh góc quay `/custom_x500/gimbal/yaw_cmd`, `pitch_cmd` | Duy nhất. Đã tách hoàn toàn khỏi node ROS từ Phase 4A. |
| 9 | **Drone Flight Control & PX4 Interface** | `src/uav_control/src/altitude_control.cpp` (`AltitudeControlNode`) | `/fmu/out/vehicle_local_position_v1` | Offboard mode, `/fmu/in/trajectory_setpoint` ($Z=-h$, giữ $X, Y$) | Duy nhất. Drone chỉ cất cánh và giữ vị trí tĩnh, chưa bay theo target. |
| 10 | **Simulation Orchestration** | `launch_simulation.sh`<br>`simulation/run_sim_phase3.sh` | CLI flags (`--headless`, `--offboard`) | Khởi chạy MicroXRCEAgent, Gazebo, Bridge, PX4 SITL | `simulation/run_sim_phase1.sh` là script đơn lẻ cũ từ Phase 1. |

---

## 3. Bảng Phân Loại Toàn Bộ File Dự Án (File Classification Table)

*Quy ước danh mục:* RUNTIME, CONFIG, LAUNCH, INTERFACE, SIMULATION, TEST, TOOLING, BENCHMARK, DOC, PROCESS, GENERATED, MODEL_ASSET.  
*Quy ước hành động:* KEEP, MOVE, MERGE, DELETE_CANDIDATE, REVIEW.

| Đường dẫn (Path) | Category | Được tham chiếu bởi (Referenced by) | Mục đích hiện tại (Current purpose) | Hành động đề xuất | Lý do kỹ thuật (Reason) |
|---|---|---|---|---|---|
| `AGENTS.md` | PROCESS | Antigravity AI context, developers | Quy tắc thay đổi mã nguồn, ranh giới an toàn cho agent | **KEEP** | Quy tắc cốt lõi của agent trong repo, hướng dẫn an toàn hệ thống. |
| `PROGRESS.md` | DOC | Developers, AI agents, README | Bảng theo dõi tiến độ, baseline nghiệm thu và tồn đọng | **KEEP** | Nguồn sự thật (source of truth) về trạng thái dự án. |
| `README.md` | DOC | Developers, git viewers | Hướng dẫn cài đặt, khởi chạy và kiến trúc tổng quan | **KEEP** | Tài liệu chào đón chính; cần cập nhật đồng bộ số lượng test. |
| `.gitignore` | CONFIG | Git | Bỏ qua thư mục build, install, log, weights, cache, runs | **KEEP** | Duy trì vệ sinh git repo. |
| `benchmark_evaluator.py` | BENCHMARK | README.md, docs/notes | Symlink trỏ vào `evaluation/benchmark_evaluator.py` | **KEEP** | Giúp chạy lệnh đánh giá trực tiếp từ root thuận tiện. |
| `metric_calculator.py` | BENCHMARK | README.md, docs/notes | Symlink trỏ vào `evaluation/metric_calculator.py` | **KEEP** | Tiện ích gọi thư viện tính toán từ root. |
| `launch_simulation.sh` | LAUNCH | README.md, `launch_phase4a_review.sh` | Script khởi chạy toàn bộ stack mô phỏng tích hợp | **KEEP** | Điểm vào chính để chạy mô phỏng Phase 3+. |
| `launch_phase4a_review.sh` | LAUNCH | Developers | Script mở mô phỏng, perception và rqt trong WSL terminal | **KEEP** | Công cụ review trực quan nghiệm thu Phase 4A. |
| `config/sim/gazebo_rendering.sh` | CONFIG | `launch_simulation.sh`, `run_sim_phase3.sh` | Cấu hình Mesa Gallium D3D12 GPU cho Gazebo trên WSL2 | **KEEP** | Tối ưu hóa GPU phần cứng, đưa camera FPS từ 5 lên 30 FPS. |
| `config/sim/ros_gz_bridge.yaml` | CONFIG | `launch_simulation.sh`, `run_tracking_3_people_test.sh` | Cấu hình bridge topic giữa Gazebo Harmonic và ROS 2 | **KEEP** | Thành phần cầu nối quan trọng cho camera và gimbal. |
| `docs/system_contract.md` | DOC | AGENTS.md, PROGRESS.md | Hợp đồng giao tiếp ROS 2 (topics, QoS, camera axis) | **KEEP** | Tài liệu đặc tả kỹ thuật nền tảng. |
| `docs/project_overview.md` | DOC | Developers | Thiết kế kiến trúc tổng thể 5 giai đoạn | **KEEP** | Tài liệu kiến trúc toàn dự án. |
| `docs/tracking/botsort_migration_spec.md` | DOC | Task BoT-SORT tiếp theo | Đặc tả chi tiết di chuyển từ DeepSORT sang BoT-SORT + GMC | **KEEP** | Tài liệu hướng dẫn thiết kế cho phase tiếp theo. |
| `docs/simulation/gimbal_camera_phase1.md` | DOC | Lịch sử Phase 1 | Báo cáo hoàn thành mô phỏng gimbal và camera | **KEEP** | Lưu trữ lịch sử mốc dự án (theo quy tắc AGENTS.md không sửa). |
| `docs/simulation/phase2_altitude_control.md` | DOC | Lịch sử Phase 2 | Báo cáo hoàn thành điều khiển độ cao Offboard | **KEEP** | Lưu trữ lịch sử mốc dự án. |
| `docs/simulation/phase3_integration.md` | DOC | Lịch sử Phase 3 | Báo cáo hoàn thành tích hợp full stack mô phỏng | **KEEP** | Lưu trữ lịch sử mốc dự án. |
| `docs/notes/phase4a_audit_2026-10-14.md` | DOC | Đội ngũ phát triển | Kết quả audit kỹ thuật Phase 4A sáng ngày 2026-10-14 | **KEEP** | Bằng chứng đánh giá kỹ thuật quan trọng của Phase 4A. |
| `docs/notes/deepsort_uav_deep_research_2026-10-12.md` | DOC | Nghiên cứu thuật toán | Khảo sát chuyên sâu DeepSORT và BoT-SORT trên UAV | **KEEP** | Cơ sở lý thuyết cho việc chọn BoT-SORT + GMC. |
| `docs/notes/uav_tracking_landscape_2026-10-13.md` | DOC | Nghiên cứu thị trường | Tổng quan các kiến trúc tracking UAV phổ biến hiện nay | **KEEP** | Tài liệu tham khảo nghiên cứu. |
| `docs/notes/uav_tracking_multiplatform_2026-10-13.md` | DOC | Nghiên cứu đa nền tảng | Phân tích giải pháp tracking đa nền tảng phần cứng | **KEEP** | Tài liệu tham khảo nghiên cứu. |
| `docs/notes/phase4a_final_review_2026-10-13.md` | DOC | Nghiên cứu Phase 4A | Báo cáo đánh giá cuối cùng Phase 4A candidate final6 | **KEEP** | Bằng chứng lịch sử nghiệm thu. |
| `docs/notes/phase4a_final_review_evidence_2026-10-13.md` | DOC | Nghiên cứu Phase 4A | Bảng chứng cứ chi tiết từng frame cho final6 | **KEEP** | Bằng chứng lịch sử nghiệm thu. |
| `docs/notes/phase4a_motion_extent_spec_2026-10-13.md` | DOC | `target_identity.py` | Đặc tả kỹ thuật bảo vệ lịch sử vận tốc khi bbox bị cắt | **KEEP** | Cung cấp rationale cho thuật toán extent guard dòng 511. |
| `docs/notes/phase4a_review_2026-10-13.md` | DOC | Nghiên cứu Phase 4A | Báo cáo review trung gian candidate final5 | **KEEP** | Bằng chứng lịch sử nghiệm thu. |
| `docs/notes/tracking_identity_2026-10-09.md` | DOC | Lịch sử Phase 4A | Ghi chú thiết kế tầng Target Identity Manager ban đầu | **KEEP** | Tài liệu thiết kế lịch sử. |
| `docs/notes/tracking_phase4a_completion_2026-10-09.md` | DOC | Lịch sử Phase 4A | Báo cáo hoàn thành bước 1 của Phase 4A | **KEEP** | Lưu trữ lịch sử mốc dự án. |
| `docs/notes/project_audit_2026-10-06.md` | DOC | Lịch sử audit | Báo cáo audit toàn diện dự án ngày 06/10 | **KEEP** | Lưu trữ lịch sử audit. |
| `docs/notes/project_audit_followup_2026-10-06.md` | DOC | Lịch sử audit | Báo cáo giải quyết các điểm lưu ý của audit 06/10 | **KEEP** | Lưu trữ lịch sử audit. |
| `docs/notes/project_audit_2026-10-13.md` | DOC | Lịch sử audit | Báo cáo audit ngày 13/10 | **KEEP** | Lưu trữ lịch sử audit. |
| `docs/notes/camera_clearance_2026-10-07.md` | DOC | Thiết kế mô hình | Ghi chú sửa va chạm góc nhìn camera với càng đáp | **KEEP** | Tài liệu kỹ thuật phần cứng mô phỏng. |
| `docs/notes/gazebo_fps_diagnosis_2026-10-07.md` | DOC | Tối ưu hóa | Chẩn đoán hiện tượng tụt FPS trong Gazebo | **KEEP** | Tài liệu kỹ thuật tối ưu simulation. |
| `docs/notes/gazebo_gpu_2026-10-06.md` | DOC | Tối ưu hóa | Kết quả benchmark kích hoạt GPU D3D12 | **KEEP** | Bằng chứng tối ưu hóa hiệu năng đồ họa. |
| `docs/notes/upright_fullbody_2026-10-07.md` | DOC | Thiết kế góc camera | Phân tích góc nghiêng camera 70 độ để thấy toàn thân người | **KEEP** | Rationale cho việc tăng giới hạn pitch lên 70 độ. |
| `docs/notes/wsl_clock_fix_2026-10-07.md` | DOC | Cấu hình hệ thống | Hướng dẫn khắc phục lệch xung nhịp WSL và Windows Hyper-V | **KEEP** | Rất quan trọng khi chạy ROS 2 thời gian thực trên WSL2. |
| `docs/research/` | DOC | Không ai tham chiếu | Thư mục trống tạo từ 23/09 | **DELETE_CANDIDATE** | Thư mục rỗng, không chứa dữ liệu. |
| `docs/setup/` | DOC | Không ai tham chiếu | Thư mục trống tạo từ 23/09 | **DELETE_CANDIDATE** | Thư mục rỗng, không chứa dữ liệu. |
| `src/uav_control/CMakeLists.txt` | CONFIG | colcon | Cấu hình biên dịch gói C++ `uav_control` | **KEEP** | File build bắt buộc của ROS 2 C++. |
| `src/uav_control/package.xml` | INTERFACE | colcon, rosdep | Khai báo dependency gói `uav_control` | **KEEP** | Manifest bắt buộc của ROS 2 package. |
| `src/uav_control/README.md` | DOC | Developers | Hướng dẫn node `altitude_control` | **KEEP** | Tài liệu nội bộ gói điều khiển. |
| `src/uav_control/src/altitude_control.cpp` | RUNTIME | `CMakeLists.txt` | Node C++ cất cánh và giữ tọa độ Offboard | **KEEP** | Thành phần production cốt lõi tầng flight control. |
| `src/uav_control/test/test_altitude_control.cpp` | TEST | `CMakeLists.txt` | 7 unit tests kiểm tra an toàn telemetry và heading | **KEEP** | Bộ kiểm thử an toàn điều khiển bay. |
| `src/uav_perception/package.xml` | INTERFACE | colcon, rosdep | Khai báo dependency gói Python `uav_perception` | **KEEP** | Manifest bắt buộc của ROS 2 package. |
| `src/uav_perception/setup.py` | CONFIG | colcon | Cấu hình cài đặt package Python và console scripts | **KEEP** | File cài đặt bắt buộc của ROS 2 python. |
| `src/uav_perception/setup.cfg` | CONFIG | colcon | Chỉ định thư mục cài đặt `lib/uav_perception` | **KEEP** | File cấu hình cài đặt ROS 2. |
| `src/uav_perception/resource/uav_perception` | INTERFACE | ament | Marker file index của ament resource | **KEEP** | Bắt buộc cho ROS 2 package discovery. |
| `src/uav_perception/README.md` | DOC | Developers | Tài liệu hướng dẫn node nhận diện và bám mục tiêu | **KEEP** | Cần cập nhật trạng thái Phase 4A mới nhất. |
| `src/uav_perception/uav_perception/__init__.py` | RUNTIME | Python imports | Export các class cốt lõi (`TargetIdentityManager`, v.v.) | **KEEP** | Python module export. |
| `src/uav_perception/uav_perception/yolo_detector.py` | RUNTIME | `setup.py`, ROS 2 launch | Node chính tích hợp YOLO26, DeepSORT, Guard, Gimbal | **KEEP** | Node production cốt lõi tầng perception. |
| `src/uav_perception/uav_perception/target_identity.py` | RUNTIME | `yolo_detector.py`, unit tests | FSM quản lý danh tính mục tiêu, 7 cổng kiểm tra, extent guard | **KEEP** | Module cốt lõi bảo vệ danh tính mục tiêu. |
| `src/uav_perception/uav_perception/gimbal_controller.py` | RUNTIME | `yolo_detector.py`, unit tests | Bộ điều khiển visual servoing 2-DOF (yaw, pitch) | **KEEP** | Module cốt lõi điều khiển góc gimbal. |
| `src/uav_perception/uav_perception/image_utils.py` | RUNTIME | `yolo_detector.py`, unit tests | Giải mã ảnh ROS, padding step, HSV torso descriptor | **KEEP** | Tiện ích xử lý ảnh chuẩn hóa. |
| `src/uav_perception/uav_perception/detection_utils.py` | RUNTIME | `yolo_detector.py`, unit tests | Đóng gói raw detection sang Detection2DArray | **KEEP** | Tiện ích định dạng thông điệp ROS. |
| `src/uav_perception/uav_perception/telemetry_utils.py` | RUNTIME | `yolo_detector.py`, evaluator | Đóng gói latency/timestamp mảng Float64MultiArray | **KEEP** | Tiện ích phục vụ đo đạc hiệu năng thời gian thực. |
| `src/uav_perception/test/test_*.py` (7 files) | TEST | pytest | 67 unit tests kiểm thử logic perception | **KEEP** | Bảo vệ chất lượng code tầng perception (PASS 100%). |
| `evaluation/benchmark_evaluator.py` | BENCHMARK | Root symlink, tests | Node ROS 2 đánh giá benchmark thời gian thực | **KEEP** | Công cụ đánh giá chính thức của hệ thống. |
| `evaluation/metric_calculator.py` | BENCHMARK | `benchmark_evaluator.py`, tests | Thư viện tính toán chỉ số bám và độ trễ | **KEEP** | Công cụ toán học cốt lõi cho benchmark. |
| `evaluation/frame_matcher.py` | BENCHMARK | `benchmark_evaluator.py`, tests | Buffer trượt ghép cặp timestamp chính xác nanosecond | **KEEP** | Xử lý dữ liệu đến lệch pha giữa camera và detection. |
| `evaluation/evaluate_phase4a_target_guard.py` | BENCHMARK | `test_phase4a_target_guard_evaluator.py` | Bộ đánh giá cổng nghiệm thu Phase 4A với nhãn GT | **KEEP** | Cổng nghiệm thu tự động cho Target Identity Guard. |
| `evaluation/profile_phase4a_target_guard.py` | TOOLING | Replay scripts | Profiler đo đạc chi tiết thời gian GPU/CPU từng module | **KEEP** | Phân tích hiệu năng chuyên sâu. |
| `evaluation/replay_phase4a_target_guard.py` | TOOLING | Developers | Replay nhanh logic `target_identity.py` từ cache NPZ | **KEEP** | Tăng tốc độ debug không cần chạy lại mô phỏng. |
| `evaluation/annotate_phase4a_target_events.py` | TOOLING | Dữ liệu GT | Script trích xuất và sinh nhãn ground truth từ video raw | **KEEP** | Công cụ sinh tập dữ liệu đánh giá. |
| `evaluation/check_tracking_reentry.py` | TOOLING | Kịch bản test E/F | Kiểm tra mục tiêu rời khỏi màn hình và tái nhập | **KEEP** | Xác minh trường hợp Re-entry sau khi mất dấu. |
| `evaluation/render_identity_comparison.py` | TOOLING | Trực quan hóa | Render video so sánh kết quả giữa raw track và guarded ID | **KEEP** | Tạo video đối chứng cho con người đánh giá. |
| `evaluation/tracking_loss_intervals.py` | TOOLING | `audit_tracking_identity.py` | Helper đóng khoảng thời gian mất dấu mục tiêu | **KEEP** | Tiện ích nhỏ dùng chung cho phân tích dữ liệu. |
| `evaluation/audit_tracking_identity.py` | TOOLING | `test_audit_tracking_identity.py` | Script phân tích `observations.npz` đo đạc ID switch | **KEEP** | Công cụ phân tích offline tập dữ liệu mẫu. |
| `evaluation/audit_camera_fps_accounting.py` | TOOLING | `test_camera_audit_export.py` | Script kiểm toán FPS và RTF camera Gazebo (06/10) | **REVIEW** | Đã hoàn thành nhiệm vụ lịch sử; lưu giữ phục vụ tái kiểm tra. |
| `evaluation/fps_frame_accounting_audit_report.md` | DOC | Báo cáo kiểm toán | Báo cáo kết quả kiểm toán FPS camera (06/10) | **KEEP** | Bằng chứng đánh giá hiệu năng camera lịch sử. |
| `evaluation/tracking_3_people_test.py` | TOOLING | `run_tracking_3_people_test.sh` | Node ROS 2 quan sát và ghi dữ liệu dataset 3 người | **KEEP** | Script sinh ra tập dữ liệu canonical `tracking_3_people`. |
| `evaluation/tracking_identity_candidate.py` | BENCHMARK | `replay_tracking_identity.py`, tests | Prototype thử nghiệm ban đầu (subclass DeepSORT) | **REVIEW** | Đã bị thay thế bởi `src/.../target_identity.py`; giữ để đối chứng. |
| `evaluation/tracking_appearance.py` | BENCHMARK | `tracking_3_people_test.py` | Helper trích xuất HSV ban đầu cho prototype | **REVIEW** | Đã bị thay thế bởi `src/.../image_utils.py`. |
| `evaluation/replay_tracking_identity.py` | TOOLING | Developers | Script chạy replay cho prototype cũ | **REVIEW** | Đi kèm với `tracking_identity_candidate.py` để so sánh baseline. |
| `evaluation/test_*.py` (7 files) | TEST | pytest | Các bộ kiểm thử cho evaluation suite | **KEEP** | Bảo vệ tính chính xác của công cụ đánh giá (PASS 100%). |
| `evaluation/annotations/` | BENCHMARK | Evaluator scripts | Chứa nhãn ground truth CSV/JSON và ảnh kiểm duyệt | **KEEP** | Dữ liệu nhãn chuẩn để chấm điểm hệ thống. |
| `evaluation/results/` | BENCHMARK | Báo cáo nghiệm thu | Chứa kết quả baseline 25s và các thư mục chạy `runs/` | **KEEP** | Bằng chứng thực nghiệm. |
| `simulation/models/` | SIMULATION | Gazebo Harmonic | 5 thư mục models (custom drone, target car, persons) | **KEEP** | Tài nguyên mô hình 3D cho Gazebo. |
| `simulation/worlds/` | SIMULATION | Gazebo Harmonic | 3 thế giới mô phỏng SDF (`gimbal_test`, 3-people tests) | **KEEP** | Cấu hình môi trường vật lý mô phỏng. |
| `simulation/run_sim_phase1.sh` | SIMULATION | Lịch sử Phase 1 | Script chạy Gazebo đơn lẻ không qua ROS | **REVIEW** | Đã được thay thế hoàn toàn bởi `launch_simulation.sh`. |
| `simulation/run_sim_phase3.sh` | SIMULATION | `launch_simulation.sh` | Script chạy Gazebo Harmonic tích hợp PX4 plugins | **KEEP** | Được gọi trực tiếp bởi all-in-one launcher. |
| `simulation/run_tracking_3_people_test.sh` | SIMULATION | Benchmark runners | Tự động hóa chạy kịch bản 3 người cắt nhau | **KEEP** | Công cụ chạy benchmark tích hợp. |
| `skills/` (3 directories) | PROCESS | AGENTS.md | Các quy trình bug-fix, runtime verification, phase review | **KEEP** | Quy trình kỹ thuật hiện hành. |
| `.ua/` | TOOLING | Công cụ Understand-Anything | Dữ liệu knowledge graph và dashboard quan sát code | **REVIEW** | Dữ liệu sinh bởi công cụ ngoài; nên đưa vào `.gitignore`. |
| `project/~` | GENERATED | Tạo nhầm | Thư mục sinh ra do shell không expand `~` | **DELETE_CANDIDATE** | File thừa (chứa bản copy `yolo26n.pt` 5.4MB). |
| `../yolo26n.pt` | MODEL_ASSET | Tạo nhầm ở root | Bản copy trọng số YOLO tại `/home/tungt/UAV_Project/yolo26n.pt` | **DELETE_CANDIDATE** | File mồ côi (orphaned), node dùng cache `~/.cache/`. |
| `../log/` | GENERATED | Tạo nhầm ở root | Thư mục colcon log sinh ra tại `/home/tungt/UAV_Project/log/` | **DELETE_CANDIDATE** | Log chạy nhầm vị trí thư mục gốc. |

---

## 4. Danh Sách Phân Nhóm Dọn Dẹp (Cleanup Candidates)

### Nhóm 1: `DELETE_CANDIDATE` (Ứng viên xóa an toàn khi người dùng phê duyệt)
1. **`/home/tungt/UAV_Project/project/'~'/`** (5.4 MB): Thư mục lỗi tạo ra do shell script chạy với chuỗi `~` không mở rộng, chứa bản copy thừa của `yolo26n.pt`.
2. **`/home/tungt/UAV_Project/yolo26n.pt`** (5.4 MB): File trọng số mồ côi ở thư mục root ngoài cùng, hoàn toàn không được tham chiếu (node lấy từ `~/.cache/ultralytics/`).
3. **`/home/tungt/UAV_Project/log/`** (~4 KB): Log colcon tạo nhầm ở thư mục ngoài cùng thay vì trong `project/`.
4. **`project/docs/research/`** (0 B): Thư mục rỗng từ ngày 23/09.
5. **`project/docs/setup/`** (0 B): Thư mục rỗng từ ngày 23/09.

### Nhóm 2: `REVIEW` (Cần người dùng xác nhận mục đích lưu trữ)
1. **`project/.ua/`** (720 KB): Dữ liệu phân tích knowledge graph từ Understand-Anything. Nếu người dùng không dùng dashboard này thường xuyên, nên bổ sung vào `.gitignore`.
2. **`simulation/run_sim_phase1.sh`**: Script mô phỏng Phase 1 đơn lẻ. Đã được thay thế hoàn toàn bởi `simulation/run_sim_phase3.sh` và `launch_simulation.sh`. Cân nhắc chuyển vào thư mục lưu trữ `simulation/archive/`.
3. **`evaluation/tracking_identity_candidate.py`, `tracking_appearance.py`, `replay_tracking_identity.py`**: Các file prototype thử nghiệm ban đầu. Hiện production đã chuyển sang `src/uav_perception/uav_perception/target_identity.py` và `image_utils.py`. Cần đánh dấu ghi chú rõ là `[EXPERIMENTAL BASELINE]` để tránh nhầm lẫn với mã nguồn chính thức.
4. **`worktrees/perception-fresh-observation`** (32 MB, nằm ở thư mục cha): Git worktree cũ từ ngày 06/10, chứa phiên bản code cũ. Cần hỏi người dùng có muốn prune bằng `git worktree remove` không.

### Nhóm 3: `MOVE` / `MERGE` (Đề xuất tổ chức lại cấu trúc trong tương lai)
1. **Gom các script chạy mô phỏng**: Chuyển các launcher phụ hoặc script lịch sử vào thư mục chuyên biệt thay vì để rải rác.
2. **Phân tách evaluation**: Tách `evaluation/` thành 2 phần rõ rệt:
   - `evaluation/tools/`: Các công cụ benchmark chính thức (`benchmark_evaluator.py`, `metric_calculator.py`, `evaluate_phase4a_target_guard.py`).
   - `evaluation/experiments/`: Các script prototype và dữ liệu thử nghiệm lịch sử.

---

## 5. Đánh Giá Tài Liệu & File Quy Trình (Docs & Process Audit)

### Đánh giá file `AGENTS.md`:
- **Codex / Antigravity có cần file này không?**  
  **CÓ, RẤT CẦN.** File này đóng vai trò "Hiến pháp vận hành" (Operating Constitution) cho AI Agent trong repo này. Nó vạch rõ:
  1. Thư mục `project` là Git root; thư mục `../software` là vùng cấm chạm vào.
  2. Nguyên tắc thay đổi tối thiểu, không tự ý mở rộng phạm vi (scope explosion).
  3. Quy tắc không xóa file/symlink của người dùng khi chưa có lệnh.
  4. Quy trình sửa lỗi nghiêm ngặt: `REPRODUCE -> CAPTURE EVIDENCE -> FIX -> REPRODUCE AGAIN`.
- **Có chứa lệnh test/setup quan trọng không?**  
  Không chứa trực tiếp lệnh bash cụ thể, nhưng liên kết và quy định việc sử dụng các skill trong `skills/` và các tài liệu chuẩn (`PROGRESS.md`, `system_contract.md`).
- **Có bị trùng lặp với file process khác không?**  
  Không trùng lặp. Nó là mục lục cấp cao (top-level rules), trong khi các file `skills/*/SKILL.md` là các quy trình thao tác chuẩn chi tiết từng bước.
- **Hành động đề xuất**: **`KEEP`**. Giữ nguyên vẹn.

### Đánh giá các tài liệu khác:
- **`PROGRESS.md`**: Canonical document về trạng thái kỹ thuật. **`KEEP`** (cần cập nhật kết quả sau mỗi mốc).
- **`README.md`**: Canonical user guide. **`KEEP`** (cần cập nhật test count từ 14 lên 130 và pitch limit từ 60 lên 70 độ để đồng bộ với mã nguồn).
- **`docs/system_contract.md`**: Hợp đồng giao tiếp ROS 2 bất biến. **`KEEP`**.
- **`docs/tracking/botsort_migration_spec.md`**: Bản đặc tả cho task tiếp theo. **`KEEP`**.
- **`docs/simulation/*.md`**: Báo cáo mốc hoàn thành Phase 1-3. **`KEEP`** (lưu trữ bằng chứng lịch sử).
- **`docs/notes/*.md`**: 18 bài ghi chú nghiên cứu và audit. Rất có giá trị về mặt kỹ thuật và bằng chứng tái hiện lỗi. **`KEEP`**.

---

## 6. Kiểm Toán Mã Nguồn Tracker (Tracker Legacy Audit)

| Thuật toán / Thư viện | Trạng thái hiện tại trong codebase | Vị trí xuất hiện cụ thể | Đánh giá & Rủi ro khi nâng cấp |
|---|---|---|---|
| **DeepSORT** | **ĐANG CHẠY RUNTIME CHÍNH THỨC** | `src/uav_perception/uav_perception/yolo_detector.py` (L24, L151, L330-346)<br>`evaluation/tracking_3_people_test.py`<br>`evaluation/replay_phase4a_target_guard.py`<br>`evaluation/tracking_identity_candidate.py` | Sử dụng thư viện ngoài `deep_sort_realtime.deepsort_tracker.DeepSort`. Đã được bọc bởi `TargetIdentityManager` ở tầng trên để chống nhảy ID. |
| **BoT-SORT** | **CHƯA CÓ TRONG CODE RUNTIME (CHỈ CÓ ĐẶC TẢ)** | `docs/tracking/botsort_migration_spec.md`<br>`docs/notes/deepsort_uav_deep_research_2026-10-12.md` | Đã có trong thư viện Ultralytics (`ultralytics.trackers.bot_sort.BOTSORT`). Kế hoạch di chuyển đã sẵn sàng nhưng chưa được code vào `yolo_detector.py`. |
| **StrongSORT** | **KHÔNG TỒN TẠI IMPLEMENTATION** | Chỉ nhắc đến như tài liệu tham khảo nghiên cứu trong `docs/notes/tracking_identity_2026-10-09.md:75`. | Không có mã nguồn hay cấu hình nào liên quan. |
| **ByteTrack** | **KHÔNG TỒN TẠI IMPLEMENTATION** | Chỉ xuất hiện trong các bài khảo sát văn liệu (`deepsort_uav_deep_research_2026-10-12.md`, `uav_tracking_landscape_2026-10-13.md`). | Không có mã nguồn nào liên quan. |
| **GMC / CMC** (Bù chuyển động camera) | **CHƯA CÓ IMPLEMENTATION** | `docs/tracking/botsort_migration_spec.md` | Hiện tại UAV chỉ bay hover tĩnh, gimbal quay theo P-controller nên chưa có thuật toán bù affine transform của camera. BoT-SORT sẽ bổ sung tính năng này. |
| **Re-ID** | **ĐANG CHẠY RUNTIME** | 1. Neural MobileNet: DeepSORT embedder (`yolo_detector.py:336`)<br>2. Color Torso: `src/uav_perception/uav_perception/image_utils.py` (`torso_histogram`) | Tầng Target Guard dùng kết hợp cả Cosine similarity của ReID vector và Bhattacharyya similarity của HSV Torso vector để bảo vệ mục tiêu. |

---

## 7. Kiểm Toán Hệ Thống Kiểm Thử (Test Suite Audit)

Toàn bộ **141 unit tests** trong dự án hiện tại đều đạt trạng thái **PASS 100%**:

```text
============================== 130 passed in 10.98s ============================== (pytest uav_perception + evaluation)
[==========] 7 tests from 1 test suite ran. (GTest uav_control)
[  PASSED  ] 7 tests.
```

### Phân loại các file Test:

1. **Behavioral Regression Tests (Bắt buộc GIỮ NGUYÊN và MỞ RỘNG)**:
   - `src/uav_control/test/test_altitude_control.cpp` (7 tests): Kiểm tra an toàn điều khiển bay (NaN/Inf, stale telemetry, reference lock, heading alignment). **`KEEP`**.
   - `src/uav_perception/test/test_target_identity.py` (28 tests): Kiểm tra toàn bộ logic FSM, gating 7 cổng, extent guard khi bbox bị cắt. Logic này độc lập với tracker và sẽ tiếp tục dùng để bảo vệ BoT-SORT. **`KEEP`**.
   - `src/uav_perception/test/test_gimbal_controller.py` (14 tests): Kiểm tra visual servoing, EMA smoothing, deadband, rate limit, góc quay vật lý. **`KEEP`**.
   - `src/uav_perception/test/test_image_utils.py` (8 tests): Kiểm tra giải mã ảnh ROS, padding step, đổi màu BGR. **`KEEP`**.
   - `src/uav_perception/test/test_auxiliary_appearance.py` (5 tests): Kiểm tra trích xuất HSV torso histogram. **`KEEP`**.
   - `src/uav_perception/test/test_detector_only.py` (3 tests): Kiểm tra chế độ xuất bbox khi tắt tracker. **`KEEP`**.
   - `src/uav_perception/test/test_telemetry_utils.py` (4 tests): Kiểm tra mã hóa và giải mã Float64MultiArray. **`KEEP`**.
   - `evaluation/test_metric_calculator.py` (12 tests): Kiểm tra tính đúng đắn toán học của các chỉ số tracking. **`KEEP`**.
   - `evaluation/test_evaluator_matching.py` (16 tests): Kiểm tra buffer trượt ghép cặp timestamp chính xác nanosecond. **`KEEP`**.
   - `evaluation/test_phase4a_target_guard_evaluator.py` (8 tests): Kiểm tra bộ tiêu chuẩn nghiệm thu target guard. **`KEEP`**.

2. **Tracker-Specific Tests (Cần MIGRATE khi đổi sang BoT-SORT)**:
   - `src/uav_perception/test/test_target_guard_pipeline.py` (5 tests): Dùng `MockTrack` mô phỏng output của DeepSORT. Khi chuyển sang BoT-SORT, cần cập nhật adapter mô phỏng `tracked_stracks`. **`MIGRATE`**.

3. **Simulation / Tooling Tests (GIỮ LẠI)**:
   - `evaluation/test_verification_window.py` (10 tests): Kiểm tra lifecycle của benchmark evaluator trong môi trường sim-time. **`KEEP`**.
   - `evaluation/test_camera_audit_export.py` (3 tests): Kiểm tra export của script đo FPS camera. **`KEEP`**.
   - `evaluation/test_audit_tracking_identity.py` (3 tests): Kiểm tra helper phân tích dữ liệu observations. **`KEEP`**.

4. **Obsolete / Historical Tests (Cần REVIEW)**:
   - `evaluation/test_tracking_identity_candidate.py` (3 tests): Kiểm thử cho file prototype `tracking_identity_candidate.py`. Nếu giữ file prototype thì giữ test, nếu chuyển prototype vào archive thì chuyển test đi cùng. **`REVIEW`**.

---

## 8. Kiểm Toán Dữ Liệu Phát Sinh (Generated Artifacts Audit)

| Loại dữ liệu | Vị trí hiện tại | Dung lượng | Tình trạng trong `.gitignore` | Đề xuất hành động |
|---|---|---|---|---|
| **Thư mục chạy benchmark (`runs/`)** | `project/evaluation/results/runs/` (58 thư mục) | ~1.6 GB | Đã có trong `.gitignore:4` | Giữ nguyên các run nền tảng như `tracking_3_people_20261009T084549Z` (chứa `raw_camera.mp4`, `observations.npz` cho replay). Các run trung gian khác có thể nén archive nếu cần giải phóng đĩa. |
| **Build & Install artifacts** | `project/build/`, `project/install/` | ~43 MB | Đã có trong `.gitignore:1-2` | Là sản phẩm biên dịch colcon. Cần rebuild package `uav_perception` để đồng bộ source mới. |
| **Colcon Logs** | `project/log/` (57 thư mục) | ~4.7 MB | Đã có trong `.gitignore:3` | Log lịch sử colcon build/test. Có thể dọn dẹp định kỳ. |
| **Thư mục rác `'~'`** | `project/'~'/` | 5.4 MB | Đã có trong `.gitignore:17` | **Xóa bỏ** sau khi người dùng xác nhận. |
| **File mồ côi ngoài root** | `/home/tungt/UAV_Project/yolo26n.pt` | 5.4 MB | Chưa áp dụng ngoài root | **Xóa bỏ** sau khi người dùng xác nhận. |
| **Log ngoài root** | `/home/tungt/UAV_Project/log/` | 4 KB | Chưa áp dụng ngoài root | **Xóa bỏ** sau khi người dùng xác nhận. |
| **Cache Python & Pytest** | `__pycache__/`, `.pytest_cache/` | Vài trăm KB | Đã có trong `.gitignore:6-8` | Tự động tạo khi chạy test. |
| **Knowledge Graph `.ua/`** | `project/.ua/` | ~720 KB | Chưa có trong `.gitignore` | **Đề xuất thêm `.ua/` vào `.gitignore`**. |

---

## 9. Đề Xuất Cấu Trúc Thư Mục Mục Tiêu (Proposed Target Structure)

Để chuẩn hóa theo mô hình phân tầng logic (Logical Layering) nhưng **tuyệt đối tôn trọng cấu trúc ROS 2**, cấu trúc mục tiêu sau khi dọn dẹp và tổ chức lại được đề xuất như sau:

```text
/home/tungt/UAV_Project/project/
├── AGENTS.md                                   # Giữ nguyên
├── PROGRESS.md                                 # Giữ nguyên
├── README.md                                   # Giữ nguyên (đồng bộ tài liệu)
├── .gitignore                                  # Bổ sung .ua/
│
├── config/                                     # Cấu hình runtime và mô phỏng
│   └── sim/
│       ├── gazebo_rendering.sh
│       └── ros_gz_bridge.yaml
│
├── docs/                                       # Tài liệu dự án
│   ├── project_overview.md
│   ├── system_contract.md
│   ├── project_structure_audit.md              # Báo cáo audit này
│   ├── tracking/
│   │   └── botsort_migration_spec.md
│   ├── simulation/                             # Báo cáo kết quả các Phase
│   └── notes/                                  # Ghi chú kỹ thuật chuyên sâu
│
├── launch/                                     # [Đề xuất tạo mới] Gom các launcher
│   ├── launch_simulation.sh                    # (Chuyển từ root vào đây, giữ symlink ở root)
│   └── launch_phase4a_review.sh                # (Chuyển từ root vào đây, giữ symlink ở root)
│
├── simulation/                                 # Môi trường mô phỏng
│   ├── models/
│   ├── worlds/
│   └── scripts/                                # [Đề xuất gom] run_sim_phase3.sh, run_tracking_3_people_test.sh
│
├── evaluation/                                 # Bộ công cụ đánh giá & benchmark
│   ├── benchmark_evaluator.py                  # Core evaluator
│   ├── metric_calculator.py                    # Core metric library
│   ├── frame_matcher.py                        # Core timestamp matcher
│   ├── evaluate_phase4a_target_guard.py        # Acceptance gate
│   ├── profile_phase4a_target_guard.py         # Profiler
│   ├── replay_phase4a_target_guard.py          # Fast replay
│   ├── annotations/                            # Ground truth data
│   ├── results/                                # Baseline & runs
│   ├── tests/                                  # [Đề xuất gom] Các file test_*.py của evaluation
│   └── legacy/                                 # [Đề xuất gom] tracking_identity_candidate.py, tracking_appearance.py, replay_tracking_identity.py
│
├── skills/                                     # Standard Operating Procedures cho Agent
│
└── src/                                        # ROS 2 Core Packages (GIỮ NGUYÊN)
    ├── uav_control/
    └── uav_perception/
```

### Bảng Mapping Chuyển Đổi Đường Dẫn (Mapping Table):

| Đường dẫn hiện tại (Current Path) | Đường dẫn đề xuất (Proposed Path) | Trạng thái hành động |
|---|---|---|
| `launch_simulation.sh` (root) | `launch/launch_simulation.sh` (tạo symlink tại root) | MOVE & LINK |
| `launch_phase4a_review.sh` (root) | `launch/launch_phase4a_review.sh` (tạo symlink tại root) | MOVE & LINK |
| `simulation/run_sim_phase3.sh` | `simulation/scripts/run_sim_phase3.sh` | MOVE |
| `simulation/run_tracking_3_people_test.sh` | `simulation/scripts/run_tracking_3_people_test.sh` | MOVE |
| `simulation/run_sim_phase1.sh` | `simulation/archive/run_sim_phase1.sh` | MOVE (Lưu trữ) |
| `evaluation/test_*.py` | `evaluation/tests/test_*.py` | MOVE |
| `evaluation/tracking_identity_candidate.py` | `evaluation/legacy/tracking_identity_candidate.py` | MOVE (Lưu trữ đối chứng) |
| `evaluation/tracking_appearance.py` | `evaluation/legacy/tracking_appearance.py` | MOVE (Lưu trữ đối chứng) |
| `evaluation/replay_tracking_identity.py` | `evaluation/legacy/replay_tracking_identity.py` | MOVE (Lưu trữ đối chứng) |
| `docs/research/`, `docs/setup/` | N/A (Xóa thư mục rỗng) | DELETE |

---

## 10. Đánh Giá Rủi Ro (Risk Assessment)

1. **Rủi ro đứt gãy Symlink hoặc Đường dẫn tương đối khi di chuyển script**:
   - `launch_simulation.sh` và `launch_phase4a_review.sh` sử dụng biến `SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"` và tính đường dẫn tương đối đến `../software/PX4-Autopilot`. Nếu di chuyển vào `launch/`, cấp thư mục thay đổi từ `../software` thành `../../software`.
   - **Biện pháp giảm thiểu**: Giữ nguyên vị trí ở root hiện tại, hoặc nếu chuyển thì phải tạo symlink tương thích tại root và cập nhật logic tính `SCRIPT_DIR`.
2. **Rủi ro phá vỡ quy trình Replay khi di chuyển file trong `evaluation/`**:
   - Một số script như `evaluation/tracking_3_people_test.py` import trực tiếp `from tracking_appearance import ...`.
   - **Biện pháp giảm thiểu**: Chỉ di chuyển sau khi đã audit toàn bộ import; hiện tại giữ nguyên cấu trúc phẳng của `evaluation/`.
3. **Rủi ro mất dữ liệu tập mẫu nghiệm thu (Canonical Run Artifacts)**:
   - Thư mục `evaluation/results/runs/tracking_3_people_20261009T084549Z` chứa video raw `raw_camera.mp4` và mảng `observations.npz` (3,166 frames). Toàn bộ hệ thống offline profiler và replay phụ thuộc vào file này.
   - **Biện pháp giảm thiểu**: **Tuyệt đối không xóa thư mục run này** trong bất kỳ đợt dọn dẹp nào.

---

## 11. Trả Lời 6 Câu Hỏi Bắt Buộc Của Task

### 1. Runtime project hiện tại thực sự gồm những file/component nào?
Runtime thực tế bao gồm đúng **11 files cốt lõi** chia làm 3 cụm:
- **Tầng Điều Khiển Bay (`uav_control`)**:
  - `src/uav_control/src/altitude_control.cpp`: Node C++ Offboard cất cánh và giữ vị trí tĩnh.
  - `src/uav_control/CMakeLists.txt`, `package.xml`: Manifest và cấu hình biên dịch.
- **Tầng Nhận Diện & Bám Mục Tiêu (`uav_perception`)**:
  - `src/uav_perception/uav_perception/yolo_detector.py`: Node Python tích hợp điều phối toàn bộ pipeline.
  - `src/uav_perception/uav_perception/target_identity.py`: Quản lý danh tính, FSM, 7 cổng kiểm tra và extent guard chống cắt bbox.
  - `src/uav_perception/uav_perception/gimbal_controller.py`: Bộ điều khiển 2-DOF visual servoing phát lệnh góc gimbal.
  - `src/uav_perception/uav_perception/image_utils.py`: Giải mã ảnh ROS và tính HSV torso descriptor.
  - `src/uav_perception/uav_perception/detection_utils.py`: Đóng gói `vision_msgs/Detection2DArray`.
  - `src/uav_perception/uav_perception/telemetry_utils.py`: Mã hóa Float64MultiArray đo đạc độ trễ.
  - `src/uav_perception/package.xml`, `setup.py`: Manifest và cài đặt ROS 2 Python.
- **Tầng Cấu Hình & Mô Phỏng Tích Hợp**:
  - `config/sim/ros_gz_bridge.yaml`: Cầu nối ROS 2 <-> Gazebo Harmonic.
  - `config/sim/gazebo_rendering.sh`: Kích hoạt GPU D3D12.
  - `simulation/models/custom_x500_gimbal_down/model.sdf`: Mô hình drone X500 gắn gimbal 3-DOF và camera.
  - `simulation/worlds/gimbal_test.sdf`: Thế giới mô phỏng.
  - `launch_simulation.sh` & `simulation/run_sim_phase3.sh`: Script khởi chạy toàn bộ stack.

### 2. File nào chắc chắn cần giữ?
- Toàn bộ **11 files runtime** ở trên.
- Toàn bộ **14 file unit test** (`src/uav_control/test/test_altitude_control.cpp`, 7 file trong `src/uav_perception/test/`, và các file `evaluation/test_*.py`).
- Bộ công cụ đánh giá chính thức: `evaluation/benchmark_evaluator.py`, `metric_calculator.py`, `frame_matcher.py`, `evaluate_phase4a_target_guard.py`, `profile_phase4a_target_guard.py`, `replay_phase4a_target_guard.py`.
- Tập dữ liệu chuẩn: `evaluation/results/runs/tracking_3_people_20261009T084549Z/` và `evaluation/annotations/`.
- Tài liệu quy trình và giao tiếp: `AGENTS.md`, `PROGRESS.md`, `README.md`, `docs/system_contract.md`, `docs/project_overview.md`, `docs/tracking/botsort_migration_spec.md`, và thư mục `skills/`.

### 3. File nào nhiều khả năng obsolete?
- `project/'~'/`: Thư mục rác tạo nhầm (chắc chắn obsolete).
- `/home/tungt/UAV_Project/yolo26n.pt`: Trọng số thừa ở thư mục ngoài (chắc chắn obsolete).
- `/home/tungt/UAV_Project/log/`: Log thừa ở thư mục ngoài (chắc chắn obsolete).
- `docs/research/`, `docs/setup/`: Thư mục trống từ tháng 9 (obsolete).
- `simulation/run_sim_phase1.sh`: Script chạy đơn lẻ cũ, không còn tích hợp với PX4 và ROS 2 (obsolete về mặt runtime, có thể lưu trữ lịch sử).
- `evaluation/tracking_identity_candidate.py`, `tracking_appearance.py`, `replay_tracking_identity.py`: Prototype thử nghiệm ban đầu của Phase 4A (obsolete về mặt production, giữ lại làm đối chứng benchmark).

### 4. File nào cần merge/move?
- `launch_simulation.sh` và `launch_phase4a_review.sh`: Có thể move vào `launch/` (kèm symlink ở root để giữ tiện ích dòng lệnh).
- Các unit tests trong `evaluation/` (`test_*.py`): Có thể move vào `evaluation/tests/`.
- Các prototype cũ trong `evaluation/`: Có thể move vào `evaluation/legacy/`.
- `simulation/run_sim_phase1.sh`: Move vào `simulation/archive/`.

### 5. File nào chưa đủ thông tin để quyết định?
- **`project/.ua/`**: Thư mục dữ liệu sinh tự động của Understand-Anything. Cần xác nhận với người dùng xem có muốn duy trì công cụ này hay xóa đi để làm gọn repo.
- **57 thư mục chạy cũ trong `evaluation/results/runs/`** (ví dụ các bản `phase4a_dev_resume_clean_v1` đến `v10`): Cần xác nhận người dùng có muốn lưu trữ (tar/zip) ra ngoài để tiết kiệm 1.6 GB đĩa hay giữ nguyên tại chỗ.
- **Git worktree `worktrees/perception-fresh-observation`**: Nằm ngoài Git root của `project`, cần xác nhận người dùng đã hoàn thành kiểm tra nhánh này chưa trước khi xóa.

### 6. Kế hoạch dọn dẹp ngắn (Cleanup Plan) cho task tiếp theo:
1. **Bước 1 (Dọn dẹp file thừa an toàn)**: Xóa `project/'~'`, `/home/tungt/UAV_Project/yolo26n.pt`, `/home/tungt/UAV_Project/log`, và 2 thư mục rỗng `docs/research/`, `docs/setup/`. Bổ sung `.ua/` vào `.gitignore`.
2. **Bước 2 (Đồng bộ build artifacts)**: Chạy `colcon build --packages-select uav_perception` để cập nhật `install/` khớp với mã nguồn mới nhất ngày 13/10 (tránh bẫy chạy code cũ từ `install/setup.bash`).
3. **Bước 3 (Thêm COLCON_IGNORE)**: Đặt file `COLCON_IGNORE` vào thư mục snapshot `evaluation/results/runs/phase4a_target_guard_20261013_final6/source_snapshot/` để ngăn ngừa lỗi trùng tên package khi build/test toàn workspace.
4. **Bước 4 (Cập nhật đồng bộ tài liệu)**: Cập nhật số lượng unit test trong `README.md` (từ 14 lên 130 tests) và góc pitch tối đa (từ 60 lên 70 độ).
