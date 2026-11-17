# Kế hoạch dọn dẹp và sửa lỗi UAV Track

Mốc đối chiếu là commit `451c93d`. Mục tiêu là chạy demo đúng từ hướng dẫn, bảo toàn kết quả benchmark cũ và giảm số script, tài liệu phải duy trì. Tách thay đổi điều khiển bay khỏi thay đổi tài liệu và đo hiệu năng để có thể kiểm tra hoặc hoàn tác từng phần.

Khi chốt kế hoạch, worktree có thay đổi riêng trong `gpu_inference.py` quanh CUDA Graph replay. Giữ bản sửa này và dùng cùng một phiên bản của nó ở cả hai phía khi đo A/B. Các gói ROS và executable cần cho bước chuẩn bị đã được tìm thấy trong môi trường. Kiểm tra CUDA ReID và chạy toàn hệ thống vẫn là việc của lượt xác minh demo.

## 1. Sửa hướng dẫn chạy demo

**Phạm vi** `README.md`, `src/uav_perception/README.md` và các lệnh dùng đường dẫn máy cũ trong `src/uav_control/README.md`.

- [x] Dùng Git root làm mốc cho lệnh. Ghi rõ thứ tự nạp ROS Jazzy, workspace phụ thuộc và workspace dự án. Tài liệu gói perception giữ lệnh chạy đầy đủ với `use_sim_time:=true`. README chính dẫn đến lệnh đó và hướng dẫn chọn topic trong viewer.
- [x] Ghi rõ `launch_simulation.sh` khởi chạy Gazebo, bridge và PX4 SITL. Node perception chạy riêng. Ghi các điều kiện kiểm tra trước khi chạy gồm `ros_gz_bridge`, `px4_msgs`, `MicroXRCEAgent`, binary PX4, môi trường YOLO và CUDA ReID. Để launcher mở tab Windows Terminal ở phần tiện ích WSL.
- [x] Hướng dẫn dùng `/custom_x500/camera/image_raw` để quan sát video gốc và `/yolo/annotated_image` để xem kết quả AI. Nêu rõ hai topic có thể chạy ở FPS khác nhau.

**Đạt khi** có thể theo tài liệu từ checkout ở đường dẫn khác. Lượt smoke test thấy `/clock` tăng, ảnh gốc tới ROS, perception dùng sim time và phát hiện một frame mới. Nếu môi trường thiếu phụ thuộc, ghi tên phụ thuộc và bước chưa xác minh.

## 2. Bảo vệ kết quả đo khỏi ghi đè

**Phạm vi** `evaluation/audit_camera_fps_accounting.py` và `evaluation/test_camera_audit_export.py`.

- [x] Bổ sung `--output-dir`. Mặc định tạo thư mục mới dưới `evaluation/results/runs/` cho mỗi lần chạy. Tạo thư mục độc quyền trước khi ghi để hai lần chạy không dùng chung đích. Từ chối đường dẫn đích đã chứa kết quả và in vị trí CSV, JSON sau khi xuất.
- [x] Giữ tên hai file, schema, cách ghép frame và công thức. Các bản dữ liệu đã được Git theo dõi tại `evaluation/results/` tiếp tục là kết quả lịch sử.
- [x] Cập nhật test export hiện có để đưa thư mục đầu ra trực tiếp. Kiểm tra hai lần chạy mặc định tạo hai đích riêng và lần chạy trùng đích giữ nguyên nội dung cũ.

**Đạt khi** test export và nhóm test matcher đạt, hai output đọc được bằng công cụ hiện tại, và file kết quả lịch sử giữ nguyên byte. Phép đo camera nguồn dùng profiler đọc Gazebo trực tiếp. Audit hai subscriber ROS vẫn phục vụ đối chiếu đường nhận ảnh.

## 3. Kiểm tra tham số độ cao

**Phạm vi** `src/uav_control/src/altitude_control.cpp`, `src/uav_control/test/test_altitude_control.cpp` và tài liệu tham số của gói.

- [x] Dùng chung kiểm tra ở constructor và callback cập nhật tham số. Từ chối `NaN`, vô cực và giá trị vượt miền biểu diễn của trường setpoint kiểu `float` trước khi cập nhật state. Báo lý do có tên tham số. Lượt này giữ miền vận hành đang dùng cho giá trị hữu hạn, kể cả `0.0` dùng để hạ cánh. Mọi trần độ cao vận hành cần một yêu cầu SITL riêng.
- [x] Xác minh yêu cầu cập nhật nguyên tử bị từ chối giữ độ cao và tần số cũ. Thử cả khởi động với giá trị lỗi, cập nhật lúc node đang chạy, giá trị hợp lệ và `0.0`.
- [x] Chạy GTest và lượt SITL riêng. Kiểm tra cất cánh, giữ độ cao/vị trí, thay đổi độ cao hợp lệ, từ chối cập nhật lỗi và hạ cánh qua runner hiện có. Giữ chính sách Offboard và điều khiển gimbal ở lượt sửa này.

**Đạt khi** setpoint trước và sau yêu cầu lỗi vẫn hữu hạn và giữ giá trị cũ, cập nhật hợp lệ vẫn có hiệu lực, SITL xác nhận hạ cánh và thoát sạch.

## 4. Rút gọn tài liệu hiện hành

**Phạm vi** `PROGRESS.md`, các báo cáo được chọn và tài liệu dẫn tới chúng.

- [x] Rút `PROGRESS.md` còn kiến trúc đang chạy, mốc kiểm chứng có ngày, hạn chế và việc tiếp theo. Giữ 187 test CUDA/ReID là số liệu lịch sử ngày 16/10. Bỏ tiêu đề “Last updated” ngày 15/10 khỏi bản tóm tắt hiện hành. Dẫn số liệu chi tiết tới các báo cáo nguồn.
- [x] Chuyển `docs/project_structure_audit.md` và nhóm review DeepSORT ngày 13/10 gồm review, final review và evidence vào `docs/archive/pre-botsort/`. Giữ tên file, nội dung và số liệu. Sửa liên kết đến chúng, kể cả liên kết tương đối từ các trang đã chuyển.
- [x] Giữ tài liệu nghiệm thu BoT-SORT, freshness, replay và đo hiệu năng hiện tại. Giữ các báo cáo milestone trong `docs/simulation/`. Với liên kết tới capture bị Git bỏ qua, ghi rõ người đọc cần dữ liệu của lần chạy nào để tái lập.

**Đạt khi** README dẫn tới một trạng thái hiện hành duy nhất, đường dẫn tài liệu đã chuyển đều mở được, và số liệu trước/sau vẫn truy được tới báo cáo gốc.

## 5. Dọn script và dữ liệu lịch sử

**Owner cho lượt triển khai** nhóm benchmark quản lý script và nhãn, nhóm mô phỏng quản lý model và launcher. Mỗi thay đổi có diff riêng.

- [x] Chuyển `evaluation/render_identity_comparison.py` và `evaluation/check_tracking_reentry.py` vào `evaluation/archive/deepsort/`. Ghi chúng cần dữ liệu DeepSORT cũ và cập nhật lệnh trong trang lịch sử tương ứng. Giữ `audit_tracking_identity.py` cùng các test trong lượt đầu.
- [x] Giữ `simulation/run_sim_phase1.sh` và hai symlink ở Git root trong lượt đầu vì có đường dẫn đã công bố. Dùng `run_sim_phase3.sh` cho hướng dẫn Gazebo hiện hành. Giữ ba world tracking làm đầu vào replay, mỗi world đại diện cho một kịch bản đã đo.
- [x] Bỏ hai bản texture trùng trong `simulation/models/target_car/materials/textures/` sau khi rà lại liên kết SDF, OBJ và MTL. Giữ bản trong `meshes/`. Kiểm tra render vật liệu xe trong Gazebo sau thay đổi.
- [x] Đối chiếu cặp `phase4a_target_crossing_annotations.{csv,json}` và `phase4a_target_crossing_reviewed_20261013.{csv,json}`. Cặp đầu đang được công cụ gọi mặc định. Chỉ bỏ bản trùng khi lệnh replay lịch sử và ghi chú nguồn nhãn đã dẫn tới bản còn lại, với hash không đổi. Giữ bản v2 và các revision có nội dung khác.

**Đạt khi** bằng chứng lịch sử còn tra được, luồng benchmark hiện tại vượt test và world mặc định render xe đúng vật liệu.

## 6. Bỏ trạng thái trùng trong node perception

**Phạm vi** `src/uav_perception/uav_perception/yolo_detector.py` và fixture test liên quan.

- [x] Xóa các bản sao đã rà không có chỗ đọc trên node, gồm `current_observations`, `smooth_cx`, `smooth_cy` và bản sao `last_track_seen_time` nếu kiểm tra call site vẫn xác nhận. Giữ dữ liệu trong manager/controller sở hữu nó và giữ các trường đang dùng trong HUD hoặc telemetry.
- [x] Chạy lại test pipeline, reset, watchdog, freshness, lazy annotation và gimbal. Đối chiếu topic, payload và quyết định bám trên cùng đầu vào. Chỉ tính đây là giảm mã bảo trì.

**Đạt khi** đầu ra và trạng thái bám của các ca kiểm chứng giữ nguyên. Cơ chế từ chối ảnh cũ, BoT-SORT/ReID, ngưỡng nhận diện và điều khiển gimbal nằm ngoài lượt dọn này.

## 7. Đo tác động camera phụ

**Phạm vi ban đầu** dùng hai biến thể world/bridge tạm của `gimbal_test.sdf`. Một bản có camera phụ 640 × 480 ở 30 Hz, một bản tắt sensor và bridge của camera đó.

- [x] Chạy ít nhất ba cặp bật/tắt trên cùng cảnh, cùng profile DDS, CUDA, trạng thái viewer và thời gian warm-up. Giữ mọi thiết lập còn lại giống nhau. Đo RTF, FPS camera Gazebo, FPS ảnh gốc ROS, FPS AI xử lý xong, độ trễ quyết định p95. Khi đo ở chế độ viewer, ghi thêm số frame thực sự được vẽ và khoảng ngắt.
- [x] Nếu lợi ích lặp lại vượt dao động giữa các lần chạy, đưa camera phụ thành tùy chọn cho lúc xem hoạt ảnh. Kiểm tra cả hai chế độ. Nếu kết quả nằm trong dao động, ghi số đo và kết thúc bước này mà giữ world hiện tại.

**Đạt khi** có số liệu cặp chạy để quyết định và luồng camera chính giữ đúng topic cùng độ phân giải. Tối ưu ReID là việc riêng sau khi có baseline đúng. Mốc 12 AI FPS và p95 dưới 200 ms dùng để theo dõi hiệu năng, không dùng để đánh giá việc dọn tài liệu.

## Thứ tự triển khai và kiểm tra

| Nhóm | Phụ thuộc | Kiểm tra trước khi chuyển bước |
|---|---|---|
| Hướng dẫn demo và đầu ra benchmark | 1 và 2 độc lập | Smoke test cách chạy. Test export, matcher và kiểm tra file lịch sử. |
| Kiểm tra độ cao | 3 độc lập về mã nguồn | Review riêng về điều khiển. GTest và SITL với ca hạ cánh. |
| Tài liệu và dữ liệu lịch sử | Làm 4 và 5 sau khi chốt đường dẫn ở 1 và 2 | Rà liên kết, hash nhãn và render xe nếu dọn texture. |
| Perception và phép đo camera | Làm 6 riêng. Đo 7 sau khi có lệnh chạy ổn định ở 1. | Replay/test cho 6. Kết quả A/B cho 7. |

Khi triển khai, ghi kết quả từng nhóm dưới dạng PASS, FAIL hoặc NOT RUN cùng đường dẫn log. Review tích hợp theo đúng ba luồng demo, benchmark và hạ cánh. Giữ thay đổi theo từng nhóm để một mục chưa xác minh không kéo theo các mục đã đạt.

## Kết quả nghiệm thu thực tế (Verification Results)

| Nhóm | Trạng thái | Nội dung xác minh | Bằng chứng / Log |
|---|:---:|---|---|
| **Nhóm 1: Demo & Benchmark Protection (Tasks 1 & 2)** | **PASS** | - Document Git-root sourcing order & preconditions.<br>- `--output-dir` exclusive export & history preservation.<br>- Smoke test /clock, raw camera, perception sim time. | - [`evaluation/test_camera_audit_export.py`](../evaluation/test_camera_audit_export.py) (5/5 PASS)<br>- Smoke test log: `/clock` 2.4s -> 5.4s, 56 images received. |
| **Nhóm 2: Altitude Parameter Validation (Task 3)** | **PASS** | - Parameter validation for NaN, inf, > float::max.<br>- Atomic rejection preserves old setpoint & frequency.<br>- SITL takeoff 2.0m -> reject NaN/inf -> climb 3.0m -> land 0.0m. | - `./build/uav_control/test_altitude_control` (10/10 GTests PASS)<br>- SITL flight log: 2.0m hold -> 3.5m climb -> 0.00m touchdown. |
| **Nhóm 3: Document Pruning & History Cleanup (Tasks 4 & 5)** | **PASS** | - `PROGRESS.md` cleaned up; milestone reports linked.<br>- DeepSORT scripts/docs moved to `archive/`.<br>- Duplicate car textures removed (verified OBJ/MTL & Gazebo render).<br>- Duplicate label pair removed after updating refs. | - SHA256 verified identical before deletion.<br>- `gz sdf -k simulation/worlds/gimbal_test.sdf` Valid.<br>- `test_camera_audit_export.py` & `test_evaluator_matching.py` (21/21 PASS). |
| **Nhóm 4: Perception Cleanup & Camera Benchmark (Tasks 6 & 7)** | **PASS** | - Removed dead state copies (`current_observations`, `smooth_cx`, `smooth_cy`, `last_track_seen_time`).<br>- Full test suite 189 tests PASS.<br>- 3-pair A/B benchmark (6 runs) on secondary camera: RTF delta +0.1%, camera FPS delta -0.6%. | - `RUN_BOTSORT_REID_RUNTIME=1 pytest -q` (189 passed).<br>- [`docs/simulation/secondary_camera_benchmark_report.md`](../docs/simulation/secondary_camera_benchmark_report.md)<br>- Session output: `evaluation/results/runs/secondary_camera_benchmark_20261116_094142/`. |
| **Xác minh SITL & Chạy Live GUI thực tế** | **PASS** | - Runner dùng lệnh PX4 Land (`px4-commander land`).<br>- Xác minh telemetry tiếp đất (`landed: true`, `abs(z) < 0.2m`) và ngắt động cơ (`arming_state: 1`).<br>- Mở mô phỏng trực tiếp 3D GUI Gazebo và `rqt_image_view` trên màn hình desktop (`DISPLAY=:1`).<br>- UAV cất cánh 2.0m, bám đối tượng người đi bộ (HUD + green box), hạ cánh tự động, tắt động cơ sạch sẽ, 0 popup Apport crash. | - [`evaluation/verify_sitl.py`](../evaluation/verify_sitl.py) (11/11 PASS)<br>- [`launch_live_demo.sh`](../launch_live_demo.sh) (30s tracking demo, exit code 0, `/var/crash` clean). |
