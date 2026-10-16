# Audit dự án UAV — 2026-10-13

Verdict: **CHANGES REQUIRED** để nghiệm thu Phase 4A và chuyển sang tracking với camera chuyển động. Nền mô phỏng, perception và hover đã có; chưa có UAV bay đuổi mục tiêu. Lỗi D3 có bản sửa trong source và bằng chứng regression tốt hơn báo cáo final6. Blocker hiện tại là xác minh độc lập, đồng bộ source/runtime và kiểm chứng vòng kín.

Phạm vi: đọc source ROS/Python/C++, launch/config/model, evaluator/tests, tài liệu và artifacts mới nhất trong `project`; kiểm tra dependency hashes liên quan. Không audit toàn bộ source của PX4/ROS/vendor, không khẳng định hệ thống đủ điều kiện bay thật. Không sửa production, không khởi động Gazebo/PX4, không thay đổi `../software`. Áp dụng `skills/phase-review/SKILL.md`.

## 1. Trạng thái thực tế

| Thành phần | Đã có | Giới hạn bằng chứng |
|---|---|---|
| Gazebo + X500 + camera/gimbal | Model, bridge, launcher, bằng chứng GPU rendering trước đây | Chưa chạy lại full stack trong audit này |
| PX4 Offboard | Giữ XY/yaw ban đầu, điều chỉnh độ cao, kiểm tra telemetry trước takeoff | Chưa có following; cần chính sách telemetry mất sau takeoff |
| YOLO + DeepSORT | Detection, embedding, fresh measured boxes | Chi phí neural chiếm phần lớn replay |
| TargetIdentityManager | Appearance + HSV + motion, ambiguity abstain, reset, LOST terminal | Regression D3 tốt hơn; chưa có holdout mới đủ điều kiện |
| GimbalController | EMA, deadband, giới hạn góc và bước điều khiển | Bước theo frame; chưa bù chuyển động camera |
| Evaluation | Source-stamp matching, telemetry, nhãn crossing, replay/profiling | PASS phụ thuộc provenance nhãn; chưa đo toàn hệ thống hiện tại |
| 3D/TF/MotionArbiter/following | Chưa triển khai theo contract và source đã đọc | Đây là phần chưa làm, không phải lỗi khiến hover hiện tại đứng yên |

Git HEAD `dfad9dd`; lúc audit có 23 tracked modifications và 836 untracked files (nhiều file là evidence). Hai worktree cùng base commit. `evaluation/results/runs` khoảng 1,6 GB. Không nên dùng riêng `git diff` để kết luận đã review toàn bộ candidate.

## 2. Phát hiện và hướng xử lý

### F1 — BLOCKER: PASS regression không phải PASS nghiệm thu độc lập

Evidence: `evaluation/results/runs/phase4a_dev_motion_extent_neural_20261013_v1/regression_v2.json` dùng D3 với `event_role=held_out`, trong khi `docs/notes/phase4a_motion_extent_spec_2026-10-13.md` đã dùng lỗi D3 để định nghĩa bản sửa. Evaluator ở `evaluation/evaluate_phase4a_target_guard.py:173` chỉ kiểm tra role trong CSV, không biết lịch sử sử dụng dữ liệu.

Audit sao chép nhãn vào thư mục tạm, đổi mọi event B/C/D1/D2/D3 thành development rồi chạy lại evaluator trên cùng profile và raw detection: gate trở thành **INCONCLUSIVE**, metrics không đổi. Không sửa nhãn/evidence gốc. File `phase4a_development_reviewed_20261013_v2.csv` hiện có 180 dòng development, tức chưa chứa toàn bộ 215 dòng bao gồm D3.

Hướng xử lý: tạo manifest development mới gồm D3; giữ final6/v2 làm lịch sử. Đóng băng source/hash, chọn event/capture mới chưa dùng chỉnh guard, gán nhãn raw-only và đánh giá. Thư mục raw review D4–D6 đã tồn tại nhưng sự tồn tại contact sheet không chứng minh đã hoàn tất acceptance hoặc tính độc lập. Capture mới là bằng chứng mạnh hơn event khác trong cùng video.

### F2 — HIGH: source và bản chạy mặc định không đồng nhất

So sánh byte/hash cho thấy `install/uav_perception/lib/python3.12/site-packages/uav_perception/` khác source ở `target_identity.py`, `yolo_detector.py`, `image_utils.py`. Sau khi source môi trường như README, import thực tế trỏ tới bản `install` đó. Do đó có thể test source mới nhưng demo bản cũ. `launch_phase4a_review.sh` có build perception; hướng dẫn chạy trực tiếp trong README không bảo đảm build mới.

Hướng xử lý: trước runtime, build candidate vào prefix xác định, source đúng overlay, lưu `module.__file__` và SHA256 vào run manifest; xác minh chúng khớp source đã freeze. Audit giữ nguyên install để không âm thầm thay bản chạy của người dùng.

### F3 — HIGH: thiếu nghiệm thu camera/gimbal chuyển động

Motion gate dùng bbox/velocity trong image space (`target_identity.py:_motion_consistent`), không có biến đổi camera để tách chuyển động người và chuyển động ảnh. Replay cố định không chứng minh guard vẫn hoạt động khi gimbal/UAV quay. Đây là khoảng trống integration, chưa phải failure mới được tái hiện trong audit.

Hướng xử lý: sau fixed-camera acceptance, thử lần lượt camera đứng yên → yaw gimbal → pitch → hover đầy đủ. Ghi source stamps, joint angles/feedback, pose, target state, latency và thời điểm abstain. Xác định failure do ego-motion trước khi chọn motion compensation; chưa cần nhảy thẳng sang MotionArbiter.

### F4 — HIGH: validation tham số điều khiển bay chưa đầy đủ

`src/uav_control/src/altitude_control.cpp:79` nhận `target_altitude_m` mà không kiểm tra hữu hạn/miền; dòng 403 chuyển trực tiếp sang float và đổi dấu thành NED Z. Constructor cũng không validate altitude. Giá trị âm, NaN/Inf hoặc ngoài miền biểu diễn có thể tạo setpoint không phù hợp. `is_publish_rate_valid` chỉ kiểm tra khả năng tạo timer, chưa kiểm tra miền vận hành của offboard.

Hướng xử lý: định nghĩa miền altitude/rate cho simulator, validate tại startup và parameter update trước khi thay state; thêm boundary tests và SITL cho thay đổi tham số. Đây là finding từ source, chưa inject giá trị lỗi vào flight runtime.

### F5 — HIGH: mất telemetry sau takeoff chưa có chính sách rõ

`altitude_control.cpp:330` chỉ dừng startup khi `!has_taken_off_ && !is_telemetry_fresh()`. Sau takeoff node vẫn publish heartbeat/setpoint khi telemetry không cập nhật; node cũng không theo dõi VehicleStatus/command ACK để xác nhận trạng thái bay. Không thể suy rằng PX4 sẽ land chỉ từ hành vi này.

Hướng xử lý: định nghĩa hành vi giữ/thoát mode khi telemetry stale sau takeoff, phối hợp failsafe PX4 hiện cấu hình; kiểm chứng SITL bằng ngắt telemetry. Tránh đơn giản ngừng heartbeat mà chưa kiểm tra hậu quả failsafe.

### F6 — MEDIUM: gimbal phụ thuộc tốc độ xử lý frame

`gimbal_controller.py:104–112` cộng delta trực tiếp mỗi lần gọi, không dùng dt; `0.035` là rad/update như comment của node. Trần lý thuyết 30 update/s là 1,05 rad/s; 16,7 update/s còn khoảng 0,58 rad/s. EMA cũng theo frame. Thay đổi inference FPS làm thay đổi đáp ứng vòng điều khiển ngay cả khi giữ gain.

Hướng xử lý: trong giai đoạn control tiếp theo, chuyển gain/rate limit sang đơn vị theo giây với dt có giới hạn, xử lý reset/gap, rồi test ở nhiều FPS và xác minh SITL. Không chỉnh gain vội để bù một lượt benchmark.

### F7 — MEDIUM: geometry protection chỉ áp dụng khi crop bị đánh dấu dirty

Bản sửa tại `target_identity.py:508` chặn history update khi top/bottom thay đổi không giống translation, nhưng chỉ trong nhánh `not candidate.clean_crop`. Cleanliness hiện dựa overlap với detection khác, không trực tiếp phát hiện cắt bởi biên ảnh/miss detector. Vì vậy bản sửa D3 chưa chứng minh bao phủ mọi dạng bbox truncation. Bbox được accept vẫn có thể gửi tới gimbal dù không ghi history.

Hướng xử lý: mở rộng regression cho cắt biên ảnh, bbox co lại khi chỉ còn một detection, thay đổi scale thật và camera chuyển động; xác định rõ khi nào geometry đủ tin cậy cho motion và khi nào đủ tin cậy cho servo. Chỉ sửa khi có case tái hiện, giữ bằng chứng chống nhận nhầm người.

### F8 — MEDIUM: tài liệu và tiêu chí chất lượng còn thiếu đồng bộ

README ghi 14 perception tests; PROGRESS ghi 64 và 127 tests tổng, final6 là candidate mới nhất, D3 chưa hồi phục. Source đã có motion-extent fix và audit chạy được 130 Python tests. Giới hạn pitch hiện 70° trong code/model contract nhưng PROGRESS vẫn nêu 60° ở mục verified lịch sử mà chưa phân biệt rõ. `/yolo/detections` khi bật tracking thực tế xuất target đã chọn, không phải mọi track; raw detector có topic riêng.

Acceptance hiện yêu cầu ít nhất một confirmation trước/sau mỗi event; chưa có coverage tối thiểu hoặc maximum abstain duration. Một PASS hình thức có thể chưa đủ hữu dụng để tracking liên tục. Cần chốt các ngưỡng từ nhu cầu sản phẩm rồi áp dụng nhất quán, không chọn ngưỡng sau khi thấy kết quả.

### F9 — MEDIUM: độ tái lập và chẩn đoán runtime

Phần lớn guard/tests/evidence còn untracked; root diff không bao gồm chúng. Launcher bỏ stdout/stderr của Gazebo/bridge/agent vào `/dev/null`, kiểm tra readiness chủ yếu qua topic tồn tại. Topic xuất hiện không bảo đảm dữ liệu đang đến. Dễ mất bằng chứng khi lỗi chỉ xảy ra runtime.

Hướng xử lý: giữ log theo từng run, kiểm tra có message mới trong deadline; lập manifest source/config/model/versions và tách checkpoint source khỏi artifact lớn. Không xóa evidence để làm sạch Git.

## 3. D3: đã sửa đến đâu?

final6 bị bbox cắt ngắn tạo vertical velocity giả; frame 1292 appearance tốt nhưng vertical gate reject, cuối cùng LOST. Source hiện có extent guard để không đưa biến dạng một phía vào trusted history. Ba test mới bao gồm cắt top/bottom và translation dưới overlap.

| Phép đo trên nhãn v2 | final6 | Motion-extent hiện tại |
|---|---:|---:|
| Frame rõ đúng target | 127/176 | 137/176 |
| Coverage | 72,16% | 77,84% |
| D3 đúng / frame rõ | 17/31 | 27/31 |
| D3 trước / sau crossing | 11 / 0 | 11 / 11 |
| Wrong confirmations trong 215 frame nhãn | 0 | 0 |
| Xác nhận trên frame ambiguous | 0 | 0 |

Nguồn: final6 acceptance và `phase4a_dev_motion_extent_neural_20261013_v1/regression_v2.json`. Audit xác minh hash của mọi source/dependency trong manifest replay mới khớp file hiện tại; hash annotations/profile/evaluator/raw detections cũng khớp. Đây là tái đánh giá artifacts đã tồn tại, không phải neural replay mới chạy trong audit.

Replay mới chỉ có **1.330 frame**, 1.280 frame đo sau warmup; chưa phải toàn bộ capture 3.166 frame. Trong prefix này không có LOST; chưa kết luận phần sau. Processing FPS khoảng **16,74**, YOLO predict mean **26,46 ms**, embedding predict **23,50 ms**, manager **0,581 ms**. Không cộng neural sub-timing vào timing API chứa nó. Plain-control mới chỉ có 10 frame đo, không đủ kết luận performance regression. Không quy giảm FPS so với baseline lịch sử cho guard.

## 4. Kiểm chứng audit

- PASS: `PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" python -m pytest -q src/uav_perception/test evaluation` — 130 tests, 3 dependency deprecation warnings.
- PASS: build `uav_control` từ source hiện tại bằng colcon trong prefix tạm riêng, BUILD_TESTING=ON.
- PASS: 7/7 C++ `test_altitude_control` từ bản build mới, ROS domain riêng 219; không chạy flight controller hoặc simulator.
- PASS: `git diff --check` và `bash -n` cho master/review/simulation/rendering scripts đã kiểm tra.
- PASS: đối chiếu manifest hashes; import-path check tái hiện bản install cũ.
- PASS: chạy lại evaluator với role đúng trên bản nhãn tạm; xác nhận INCONCLUSIVE.
- NOT RUN: neural replay mới, full-capture mới, Gazebo/PX4 flight, moving-gimbal acceptance, full lint suite và audit toàn bộ vendor.

## 5. Thứ tự giải quyết đề xuất

1. Chốt candidate motion-extent hiện tại bằng source manifest; đồng bộ prefix chạy, kiểm tra import/hash. Cập nhật status để không tiếp tục sửa lại lỗi D3 đã có regression tốt.
2. Chuyển D3 sang development trong manifest mới, giữ evidence cũ; chốt coverage/abstain và latency budget trước phép đo tiếp theo.
3. Freeze source; hoàn tất nhãn/test event mới hoặc tốt hơn capture mới. Chạy full 3.166-frame regression và holdout thật; 0 wrong-person confirmation, xác nhận trước/sau mỗi crossing, đạt quality budget đã chốt.
4. Benchmark paired cùng input, cùng cấu hình, warmup và số frame đủ dài; tách camera FPS, callback delivery, processed FPS và latency. Tập trung profiling neural trước khi tối ưu manager dưới 1 ms.
5. Xác minh runtime theo tầng; sửa validation/failsafe control và kiểm chứng SITL trước khi mở rộng autonomous movement.
6. Khi các gate trên đạt mới triển khai camera-motion compensation, TF/3D state và thiết kế tích hợp body following/MotionArbiter.

Không cần thay DeepSORT hoặc thêm model chỉ để giải quyết bằng chứng hiện tại. Bước tạo giá trị lớn nhất là chứng minh bản sửa đang có trên đúng runtime và dữ liệu chưa dùng chỉnh code.

File tạo bởi audit: báo cáo này. Production, install, nhãn gốc, lịch sử reports và evidence cũ được giữ nguyên; không commit/reset/delete.
