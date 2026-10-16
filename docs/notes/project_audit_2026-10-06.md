# Rà soát dự án ngày 06/10/2026

**Kết luận: CHANGES REQUIRED.** Dự án có nền tảng mô phỏng và gimbal tracking; chưa hoàn thành UAV bay theo mục tiêu. Chưa thấy việc mở rộng kiến trúc quá mức trong mã hiện tại, nhưng lỗi tích hợp perception, điều khiển và cách đo benchmark cần được xử lý trước khi phát triển điều khiển bay theo mục tiêu.

## Phạm vi và giới hạn

- Đánh giá working tree tại base commit `dfad9dd`, bao gồm các thay đổi chưa commit và file mới. Không sửa mã, xóa, di chuyển file, build lại hay thay đổi phần mềm ngoài dự án.
- Rà soát toàn bộ nhóm file do dự án quản lý: source, test, package/build metadata, launcher, YAML, SDF/model metadata, tài liệu hiện tại, báo cáo lịch sử, skills và kết quả CSV/JSON. Với mesh/ảnh, kiểm tra tham chiếu, tồn tại và hash; không đánh giá chất lượng mỹ thuật từng ảnh.
- `software/` là dependency bên ngoài: khảo sát dung lượng, đọc các phần cần đối chiếu hành vi dependency; không kiểm toán từng file của PX4/ROS/Python bên thứ ba.
- `project/` khoảng 89 MiB; `software/` khoảng 13 GiB theo `du` làm tròn. Phần lớn dung lượng nằm ở dependencies, không phải mã tự viết.
- Không chạy mô phỏng Gazebo/PX4/CUDA end-to-end trong lượt này. Các kết quả runtime lịch sử không xác nhận Phase 4A hiện tại.
- Chỉ tạo báo cáo này; giữ nguyên tất cả thay đổi đã có của người dùng.

## Trạng thái thực tế

| Thành phần | Trạng thái |
|---|---|
| Gazebo world + custom X500 + gimbal/camera | Đã có source và báo cáo kiểm chứng lịch sử |
| ROS-Gazebo bridge + PX4 DDS | Đã có cấu hình/launcher |
| C++ cất cánh, giữ X/Y/yaw và độ cao | Đã có; có lỗi tham số và thiếu kiểm tra telemetry |
| YOLO + DeepSORT + gimbal servoing | Đã có; có lỗi tích hợp ảnh và trạng thái tracker |
| TargetIdentityManager + GimbalController (Phase 4A) | Source và 14 unit test có; chưa kiểm chứng end-to-end |
| Benchmark/evaluation | Có công cụ và artifact; một số kết luận đo chưa đáng tin |
| Ground-position estimator, TF đầy đủ, MotionArbiter | Chưa triển khai |
| UAV bay theo người, tìm kiếm khi mất mục tiêu | Chưa triển khai |

`PROGRESS.md` phân biệt source/test với runtime khá rõ. Bước tiếp theo nên là sửa lỗi và xác minh Phase 4A trước, rồi mới làm estimator; chưa nên dùng các số benchmark hiện tại để quyết định kiến trúc bay theo mục tiêu.

## Phát hiện cần ưu tiên

Đường dẫn bên dưới tính từ Git root `project/`; số dòng theo trạng thái kiểm tra.

### HIGH

1. **Lệnh perception trong README hiện không chạy được.** `README.md:108–114` hướng dẫn activate virtualenv rồi `ros2 run`. Executable đã cài tại `install/uav_perception/lib/uav_perception/yolo_detector:1` có shebang `/usr/bin/python3`; Python đó thiếu `torch`, `ultralytics`, `deep_sort_realtime`. Đã tái hiện đúng lệnh: `ModuleNotFoundError: No module named 'torch'`, exit 1. Launcher Phase 4A dùng Python virtualenv với `-m` nên đi qua đường khác. Sửa hướng dẫn dùng interpreter thực tế hoặc sửa cách build entry point; không cài thêm ML dependency vào hệ thống chỉ để né lỗi này.

2. **Offboard không chờ vị trí hợp lệ.** `src/uav_control/src/altitude_control.cpp:102–138` khóa ngay bản tin đầu tiên, không kiểm tra `xy_valid`, `z_valid`, `heading_good_for_control`, giá trị finite hoặc độ mới; timer gửi setpoint và yêu cầu arm kể cả chưa có telemetry. `has_local_position_` được gán nhưng không đọc. Topic xuất hiện trong launcher không chứng minh đã có dữ liệu hợp lệ. Cần gate bằng telemetry hợp lệ/mới và kiểm tra trạng thái/ACK. Đây là thiếu sót xác nhận bằng source; chưa tái hiện hành vi bay.

3. **Tham số publish rate có thể làm crash node.** `altitude_control.cpp:142` ép `publish_rate_hz_` sang `uint64_t` làm mẫu số phép `%`. Giá trị hợp lệ theo callback như `0.5` trở thành 0. Đã chạy executable trong ROS domain riêng 197, giới hạn 5 giây, không kết nối simulator: exit 136 (SIGFPE) ở tick đầu. Validation lúc khởi tạo cũng chưa có; rate bằng 0/âm/rất lớn cần kiểm tra trước khi tạo timer.

4. **Dự đoán cũ bị coi là quan sát mới.** `src/uav_perception/uav_perception/yolo_detector.py:314–316` lấy mọi track confirmed, không xét `time_since_update`; `target_identity.py:184–187` cập nhật `last_seen_time` và trạng thái TRACKING. Tái hiện bằng Track/KalmanFilter DeepSORT đã cài: mất detection 30 frame, kéo dài 12 giây vẫn TRACKING, confidence fallback 0.5. Timeout coasting chỉ bắt đầu sau khi tracker bị xóa. Cần phân biệt measurement và prediction, có chính sách actuation khi coasting.

5. **RGB/BGR sai tại biên inference/tracker.** `yolo_detector.py:277–284,313` truyền `img_rgb` vào Ultralytics ndarray API và DeepSORT mặc định `bgr=True`. Dependency đã cài thực hiện đổi BGR sang RGB, nên input bị đảo màu lần nữa. Test trích luồng xử lý xác nhận pixel đỏ RGB được truyền nguyên vào inference. Chuẩn hóa BGR cho các API này, giữ RGB cho output ROS theo encoding. Chưa đo ảnh hưởng lên độ chính xác model.

6. **Logical identity có thể chuyển sang người khác.** `target_identity.py:171–178` chọn track đầu tiên khi ID hiện tại biến mất, không association theo vị trí/appearance/thời gian. Tái hiện người A biến mất, người B rất xa xuất hiện: B được gắn cùng logical identity và coi là ID switch. Cần gate reacquisition hoặc chuyển LOST rồi chọn target mới rõ ràng.

7. **Detector metric thực chất đo output tracking.** `evaluation/benchmark_evaluator.py:340` và `audit_camera_fps_accounting.py:229` coi array tracking là raw YOLO detection. Output node tại `yolo_detector.py:362` chỉ chứa bbox target được tracker chọn; prediction có thể tồn tại khi detector không thấy người. Vì vậy zero detector misses/100% person detection không được chứng minh. Cần telemetry riêng cho raw detector và trạng thái tracking, hoặc đổi tên/chú giải metric.

8. **Latency hiện không phải thời gian xử lý.** `benchmark_evaluator.py:338` lấy thời gian callback detection trừ callback camera ở subscriber khác. Queue/callback ordering cho phép kết quả âm: artifact verification có minimum −2450.92 ms; audit −6709.20 ms. Không nên dùng các phân vị này làm inference latency. Xuất duration từ `perf_counter` trong perception; giữ callback-arrival difference như metric riêng nếu cần.

9. **Không đủ bằng chứng phân loại drop/source loss/matching failure.** `benchmark_evaluator.py:297,370` coi frame không khớp là unprocessed và hardcode `matching_failure=False`. Tái hiện detection đến trễ sau hạn buffer 0.6 giây: đã xử lý nhưng bị gắn unprocessed. Auditor coi số frame nhận tại subscriber thứ hai là số source publish (`audit_camera_fps_accounting.py:325,423`), không phát hiện shared transport loss. JSON audit có 37 orphan stamp và matching 89.12%, khác báo cáo ghi 100%. Cần phân loại unresolved, đợi drain ở boundary và đo publisher/bridge nếu muốn kết luận nguồn mất bao nhiêu frame.

### MEDIUM

10. **Đổi rate runtime không đổi timer.** `altitude_control.cpp:53–57,95–98`: callback chỉ đổi biến; timer chỉ tạo một lần. Đổi từ 10 sang 20 vẫn publish theo timer 10 Hz. Chuỗi arm cũng dựa trên 10 tick thay vì thời gian. Validate xong phải cập nhật timer hoặc không cho đổi rate runtime.

11. **Thông báo Ctrl+C hạ cánh không đúng implementation.** `launch_simulation.sh:310` nói “land and terminate”, nhưng `cleanup():122–160` chỉ TERM/KILL các tiến trình, không gửi lệnh land/chờ touchdown/disarm. Sửa thông báo cho đúng; nếu cần hạ cánh thực sự thì thiết kế và kiểm chứng luồng đó riêng.

12. **Tắt DeepSORT làm mất cả detection output.** `yolo_detector.py:311–326,362–378`: `enable_deepsort=False` khiến active tracks rỗng dù YOLO có người. Tái hiện với detection confidence 0.9: array xuất rỗng. Cần output detection-only hoặc reject/document chế độ này.

13. **Image decoder không xét row stride.** `yolo_detector.py:263,269` reshape toàn bộ data và bỏ qua `msg.step`; ảnh hợp lệ có padding làm ValueError và có thể dừng node. Tái hiện ảnh rgb8 một pixel, step=4. Gazebo hiện dùng ảnh tightly packed nên không chứng minh luồng hiện tại thường xuyên lỗi; đây là lỗi với input camera được phép cấu hình.

14. **Tắt gimbal tracking vẫn gửi góc ban đầu.** `yolo_detector.py:168–169` publish không xét flag; initial angle cũng chưa clamp. Cần validation và gate lệnh ban đầu.

15. **Thu history không giới hạn và không dùng.** `yolo_detector.py:351–354`: bốn list append mỗi lần control, không đọc/xóa/giới hạn. Xóa collection không có consumer hoặc dùng buffer hữu hạn nếu có nhu cầu phân tích.

16. **Track loss phụ thuộc FPS.** `metric_calculator.py:134` dùng gap cố định 0.20 giây. Cùng ID có mặt ở mọi frame tại 0, 0.4, 0.8 giây vẫn báo hai loss/reacquisition dù retention 100%. Camera headless 2.5–3 Hz vốn có khoảng frame lớn hơn threshold này. Đếm tracking transition; phân biệt thiếu camera với mất target.

17. **Convergence chấp nhận thiếu số frame hold ở cuối.** `metric_calculator.py:206` cắt cửa sổ bằng `min`; chỉ hai mẫu 120px rồi 40px vẫn được tính convergence dù yêu cầu hold năm frame. Phải có đầy đủ cửa sổ.

18. **Missing/NaN telemetry gây metric sai hoặc crash.** `metric_calculator.py:238–250` lọc yaw/pitch nhưng index theo sample gốc: missing yaw gây IndexError. `:280–298` không loại NaN, có thể trả drift NaN nhưng kết luận movement="NO". Không có telemetry phải trả unknown, không kết luận hover; drift cũng không chứng minh chuyển động dùng để tracking.

19. **Run thiếu thời lượng vẫn được ghi đủ.** `benchmark_evaluator.py:444–462` thoát do wall timeout rồi ghi requested duration. CSV verification lịch sử chỉ trải 18.316 giây simulation nhưng JSON nói 25.0. Ghi actual duration, completion reason và requested duration riêng.

20. **Rate gimbal là rad/frame, không phải rad/s.** `gimbal_controller.py:101–109` cộng delta mỗi frame không có dt; node đã chú thích đơn vị này. Với giới hạn 0.035 rad/frame, nominal 30 Hz và headless 3 Hz tạo trần tốc độ khác khoảng mười lần. Không phải lỗi tài liệu giấu kín, nhưng cần xác định thời gian và đơn vị trước khi nối điều khiển bay.

### LOW / INFO

- `metric_calculator.py:61–66` trả schema rút gọn khi không có frame; summary evaluator vẫn đọc `processed_frames` và có thể crash sau khi xuất JSON.
- `target_identity.py:192` coi timestamp 0 là chưa quan sát; observation lúc sim time 0 rồi mất tới time 2 trả SEARCHING thay vì LOST. Node hiện truyền wall time nên tránh trường hợp này.
- `yolo_detector.py:365,373` xuất underlying tracker ID; logical `active_target_id` chưa có consumer production và không được đưa ra topic. `class_id` nên giữ nghĩa lớp đối tượng thay vì chứa track ID.
- Contract camera tại `docs/system_contract.md:11–12` khẳng định Best Effort/depth 1 nhưng YAML bridge không khai báo QoS. Cần đối chiếu publisher runtime/default thực tế; subscriber của perception không chứng minh QoS của bridge.
- CameraInfo và joint state có sẵn ở phía Gazebo, nhưng pipeline perception chưa dùng chúng; joint state chưa được bridge vào ROS. Không có measurement feedback góc thật cho estimator. Đây là việc chưa triển khai, không phải lý do xóa các interface.
- README/package còn đường dẫn máy cụ thể; một số câu “stabilized”/“rate commands” mạnh hơn implementation: node xuất angle setpoint, chưa có bù tư thế UAV đầy đủ. Nên làm rõ giới hạn.
- `uav_control` dùng trực tiếp `rcl_interfaces` nhưng metadata chỉ khai báo `rclcpp`/`px4_msgs`; nên khai báo direct dependency khi chỉnh package.
- Unit metric pytest hiện có sáu test function, khác tài liệu nói bảy. Ba matching test và 14 perception test đã có.

## File dư và dung lượng

| File/nhóm | Bằng chứng và đề xuất |
|---|---|
| `~/.cache/ultralytics/yolo26n.pt` bên trong Git root | Thư mục tên literal `~`, không phải home. Hash trùng `../yolo26n.pt` và `/home/tungt/.cache/ultralytics/yolo26n.pt`; source dùng `expanduser` vào home thật. Ứng viên dọn rõ ràng, khoảng 5.3 MiB |
| `target_car/materials/textures/hatchback.png`, `wheels3.png` | Hash trùng hai file trong `target_car/meshes/`. OBJ gọi MTL ở meshes; MTL tham chiếu texture cạnh nó. Bản materials không có tham chiếu trong project; ứng viên dọn khoảng 0.82 MiB, xác minh render sau khi dọn |
| `target_person/materials/textures/*-unmodified.png` | Hai file không được DAE/SDF tham chiếu, khoảng 4.26 MiB. Có thể là bản gốc để chỉnh texture; xác nhận có cần giữ source asset trước khi xóa |
| `simulation/models/*/thumbnails/` | Khoảng 0.20 MiB; không được runtime SDF tham chiếu, có giá trị preview/model library. Tùy chọn giữ, không ưu tiên dọn |
| `build/`, `install/`, `log/`, `__pycache__/`, `.pytest_cache/` | Generated/cache; có thể tái tạo, nhưng install đang cần cho launcher. Khoảng 24 MiB cho build/install/log. Không xóa install mà không rebuild trước lần chạy tiếp theo |
| `docs/research/`, `docs/setup/` | Thư mục trống; ít ảnh hưởng. `docs/notes/` ban đầu trống, nay chứa báo cáo này |
| `simulation/run_sim_phase1.sh` | Không ở launch path hiện tại; vẫn được tài liệu milestone tham chiếu. Có thể giữ cho kiểm tra độc lập/historical workflow |
| `benchmark_evaluator.py`, `metric_calculator.py` ở root | Symlink tiện dụng đến evaluation; không phải hai bản code trùng. Nên giữ |
| `target_person/`, `target_car/` | Cả hai vẫn được world include; không thể gọi toàn bộ model là unused. Người đứng ở y=20 ngoài vùng test chính; ô tô không thuộc classes=[0], nhưng vẫn là cảnh/distractor |
| `observer_camera` trong world + bridge | Camera phụ luôn chạy 640x480@30 Hz để quan sát animation. Source pipeline chính không dùng; cân nhắc cho phép tắt trong benchmark để giảm render/transport, phải đo trước khi kết luận mức giảm |
| `evaluation/results/*`, báo cáo milestone/audit | Bằng chứng lịch sử, nên giữ và đánh dấu giới hạn; không xóa chỉ vì không import từ source |
| `skills/`, `AGENTS.md` | Quy trình làm việc cho agent, không thuộc runtime; không phải mã chết |
| `../software/` | PX4 ~6.2 GiB, venv ~5.8 GiB, ROS workspace ~488 MiB. Không dọn bằng cách xóa tùy ý dependency/library |

`target_person/` chiếm khoảng 28 MiB, phần lớn dung lượng simulation; asset đang được tham chiếu không nên chia nhỏ như code. Nếu cần giảm size thì tối ưu texture/mesh riêng và so sánh render.

Mã/biến không dùng: `has_local_position_` trong C++; import `math`, `BoundingBox2D`, local `t0`, `hud_bg`, `lh` trong perception; import `TrackingMetricCalculator` trong auditor; `PROJECT_DIR` và `det_sim` trong evaluator. Một số counter có ghi nhưng không xuất/report, nên hoặc dùng làm telemetry đúng nghĩa hoặc bỏ khi sửa phần đó.

## File nên chia

| File | Dòng | Ranh giới đề xuất |
|---|---:|---|
| `evaluation/benchmark_evaluator.py` | 704 | ROS ingestion; pure timestamp matcher; export/summary; chế độ chạy |
| `evaluation/audit_camera_fps_accounting.py` | 504 | ROS collection; pure accounting/report. Chia sẻ matcher với evaluator sau khi sửa semantics |
| `src/uav_perception/uav_perception/yolo_detector.py` | 479 | Decode ảnh; adapter detector/tracker; output serialization; HUD renderer. `process_frame` khoảng 181 dòng |
| `evaluation/metric_calculator.py` | 356 | Hàm tính khoảng 299 dòng: frame accounting, transitions, convergence, dynamics, telemetry |
| `simulation/models/walking_actor/model.sdf` | 1513 | Chủ yếu 373 waypoint; cân nhắc giảm keypoint nếu giữ nguyên trajectory. Không thêm generator chỉ vì file dài |
| `launch_simulation.sh` | 327 | Đủ nhỏ để giữ một file; ưu tiên sửa readiness và cleanup trước |
| `altitude_control.cpp` | 252 | Chưa cần tách thành nhiều node/file; ưu tiên validation/state logic |
| `target_identity.py`, `gimbal_controller.py` | 246 / 122 | Đã có ranh giới module hợp lý; chưa cần chia tiếp |

Không nên thêm package ROS, framework orchestration, CI/container hay abstraction chỉ để giảm số dòng. Các hàm/helper thuần trong package hiện có là bước đầu đủ dùng.

## Kiểm chứng trong lượt này

| Kiểm tra | Kết quả |
|---|---|
| Perception existing tests | PASS, 14/14 |
| Evaluation existing tests với ROS environment | PASS, 9/9 (6 metric + 3 matching) |
| Bash syntax bốn launcher/helper | PASS |
| ShellCheck mức warning bốn script | PASS |
| XML parse SDF/model metadata/DAE và asset tham chiếu | PASS; texture DAE được tham chiếu đều tồn tại |
| Source perception so với bản installed | PASS, bốn module giống nội dung |
| Lệnh README `ros2 run uav_perception yolo_detector` | FAIL, thiếu torch do interpreter hệ thống |
| C++ rate=0.5, ROS domain riêng, timeout 5s | FAIL, exit 136/SIGFPE |
| Tái hiện nhỏ bằng dependency/mock/in-memory | Xác nhận stale track, đổi người, RGB handoff, disable tracker, padded image, sim timestamp zero; metric gap/hold/missing-data/late matching |
| Gazebo/PX4/CUDA end-to-end Phase 4A | NOT RUN |

Lệnh test có thể lặp lại từ Git root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/uav_perception python3 -m pytest -p no:cacheprovider src/uav_perception/test/ -q
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider evaluation/test_metric_calculator.py evaluation/test_evaluator_matching.py -q
bash -n launch_simulation.sh
bash -n launch_phase4a_review.sh
bash -n simulation/run_sim_phase1.sh
bash -n simulation/run_sim_phase3.sh
shellcheck -S warning launch_simulation.sh launch_phase4a_review.sh simulation/run_sim_phase1.sh simulation/run_sim_phase3.sh
```

## Thứ tự xử lý đề xuất

1. Sửa đường chạy perception và lỗi validation/timer của altitude controller; bổ sung gate telemetry. Mọi sửa control cần integration evidence tương ứng.
2. Sửa RGB/BGR, observed/predicted track và association identity; kiểm chứng Phase 4A với mất detection, đổi ID và hai người.
3. Sửa telemetry/semantics benchmark rồi chạy lại; lưu requested/actual duration, lý do kết thúc, source counter, độ trễ đo từ node.
4. Tách các hàm lớn theo ranh giới trên, giữ API và test tương ứng; dọn asset/cache sau khi được phép.
5. Khi baseline đo được và ổn định, triển khai estimator với camera calibration, góc gimbal đo thật, altitude, frames và timestamps rõ ràng; sau đó mới MotionArbiter/body following.
