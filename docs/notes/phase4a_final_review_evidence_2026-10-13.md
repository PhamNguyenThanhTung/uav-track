# Phase 4A — báo cáo cuối để người dùng tự review, 2026-10-13

**CHANGES REQUIRED. Chưa đóng Phase 4A.** Bản `final6` không xác nhận sai target trong các cửa sổ đã gán nhãn, nhưng không xác nhận lại target sau crossing D3. Gate **INCONCLUSIVE**. Antigravity đã dừng; không dùng reviewer khác. Review độc lập của người dùng chưa thực hiện.

**Hiện trạng đã xác minh.** Codebase truy cập được; Git root là `project`, HEAD `dfad9dd`. Pipeline thực tế: `Image → YOLO26n → DeepSORT 1.3.2 → TargetIdentityManager → GimbalController`. YOLO xuất mọi person detection; manager chỉ nhận track confirmed/fresh, bbox đo `to_ltrb(orig=True, orig_strict=True)` và embedding MobileNet 1.280 chiều qua `get_feature()`. Kalman đã có mean/covariance, velocity theo bước frame; guard reuse hướng, tính velocity theo `Image.header.stamp` và history ngắn. Bbox được manager xác nhận mới tới gimbal và target topic. UNCERTAIN/PREDICTING/LOST không đưa bbox dự đoán tới controller.

**Thay đổi đang trong working tree.** `target_identity.py`: anchor/reference có giới hạn, motion theo thời gian nguồn, kiểm tra mọi candidate kể cả TID cũ, freeze khi crossing/candidate mơ hồ, xác nhận lại qua hai observation mới, chặn crop bẩn khi số detection giảm. `yolo_detector.py` + `image_utils.py`: một lần crop và một batch embedding DeepSORT hiện có; crop ấy được dùng thêm cho HSV torso 192 chiều trên CPU. Không thêm neural model/dependency, không thay tracker, không sửa gimbal/following. Reference chỉ cập nhật từ crop sạch; LOST terminal và reset rõ ràng, không lưu ảnh để tìm lại lâu dài.

HSV có bằng chứng hữu ích nhưng chưa giải quyết D3: cùng input/source/process, tắt HSV chỉ đúng 89/176 frame rõ và không xác nhận sau C/D2; bật HSV đúng 127/176 và xác nhận sau C/D2. Cả hai vẫn INCONCLUSIVE. Cosine/histogram/vector/IoU không phải neural inference.

**Nguyên nhân có bằng chứng.** Bản trước `final4` chọn nhầm người tại D2; `final5` veto association hợp lệ rồi LOST sớm. Các evidence cũ được giữ nguyên. Với `final6`, blocker hiện tại là motion history:

- Frame 1282→1284: chiều cao bbox target giảm 306,5→283,9→244,0 px; đỉnh bbox gần như giữ nguyên, centroid dọc đi 375,9→364,3→344,0 px. Crop đã được đánh dấu bẩn, nhưng `_accept()` vẫn ghi nó vào history vì torso overlap chưa vượt ngưỡng.
- History tạo velocity dọc **−58,06 px/s**. Frame 1292, candidate đúng có neural cosine **0,832**, HSV **0,967**, residual tổng **108,92 < 186,39 px**; riêng residual dọc **83,87 > 38,71 px**, nên `_motion_consistent()` trả false. Đây là lỗi geometry/history, không phải bằng chứng cần thay DeepSORT.
- Observation cuối: frame **1284**, 115,732 s. LOST: frame **1321**, 118,240 s; ảnh thô vẫn thấy target áo đỏ. [Ảnh failure](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/D3_failure_review.png), [diagnostic thật](../../evaluation/results/runs/phase4a_final6_d3_neural_diagnosis_20261013/D3_motion_gate_analysis.json).

Cached replay gốc qua D3 trong khi replay YOLO thật thất bại. Hai đầu vào không tương đương: MP4 decode/inference tạo bbox/features khác cache gốc. Không dùng cached replay để thay acceptance/runtime hoặc benchmark neural.

**Dataset, nhãn và nghiệm thu.** Capture có 3.166 frame, 8.871 detection, source time 36,964–234,204 s, camera cố định. Nhãn v2 rà từng frame trong B/C/D1/D2/D3: **215 frame = 176 rõ + 39 ambiguous**. Chỉ gán detection nào thuộc target áo đỏ; không yêu cầu PID cho B/C. Tách bbox thuộc target nhưng crop bẩn khỏi bbox không thể xác định owner. Rà từ ảnh thô và YOLO index ẩn danh, không dùng TID/guard output để gán nhãn; [contact sheets](../../evaluation/annotations/raw_review_20261013_v2/), [CSV/manifest](../../evaluation/annotations/phase4a_target_crossing_reviewed_20261013_v2.json). Nhãn v1 vẫn giữ nguyên.

B/C/D1/D2 là development; D3 là event-held-out trong cùng capture, guard được giữ nguyên trước khi mở kết quả D3. Sau chẩn đoán này, nếu sửa theo D3 phải chuyển D3 thành development và dùng event/capture chưa dùng chỉnh code làm test tiếp theo. Chưa có capture độc lập hoặc người thứ hai adjudicate nhãn. 8,12% PID thiếu trong báo cáo 2026-10-09 là 667/8.212 detection phân loại được trong cửa sổ chấm từ 50 s: 178 chưa có TID confirmed + 489 PID từ chối; không phải YOLO miss hay target switch.

| Event | Đúng / frame rõ | Đúng trước / sau | Xác nhận sai |
|---|---:|---:|---:|
| B | 16/32 | 4/12 | 0 |
| C | 23/33 | 13/9 | 0 |
| D1 | 20/23 | 5/9 | 0 |
| D2 | 51/57 | 19/7 | 0 |
| D3 | 17/31 | 11/0 | 0 |

Coverage **72,16%**; abstain **88/215 frame** (49 rõ + 39 ambiguous). Không có unverified confirmation hoặc wrong gimbal confirmation trong cửa sổ nhãn. Toàn capture: TRACKING 1.097, UNCERTAIN 222, PREDICTING 0, LOST 1.846, SEARCHING 1; 1.097 gimbal computations, **7 raw target-TID reassociations**. Không suy số logical person switches toàn video từ ID nội bộ luôn là `1`; ngoài các cửa sổ nhãn chưa có GT để kết luận.

Acceptance giữ nguyên: **0 confirmed target/gimbal chuyển sang người khác**, target đúng trước **và** sau mỗi crossing, không xác nhận bbox mơ hồ. TID đổi được phép; abstain không phải switch. Evaluator kiểm tra IoU ≥0,5 và detection owner phải khớp target index mạnh hơn bbox cạnh tranh với margin ≥0,05. Always-abstain/pre-only/geometry mơ hồ không được PASS. D3 thiếu xác nhận sau nên chưa đạt.

**Performance đã đo.** Mỗi lượt 3.166 frame, bỏ 50 warmup; ROS domain riêng, không có gimbal subscriber, không chạy Gazebo/PX4/flight. Đây là replay processing FPS, không phải FPS camera/DDS.

| Lượt | Mean process | FPS |
|---|---:|---:|
| Baseline lịch sử 2026-10-12 | 24,77 ms | 40,37 |
| Cùng process hiện tại, tắt HSV | 59,92 ms | 16,69 |
| final6 bật HSV, không instrumentation | 60,29 ms | 16,59 |
| final6 component profiling | 61,04 ms | 16,38 |

Chênh plain bật/tắt **+0,38 ms (~0,63%)**, là chênh tổng đo được, không phải attribution riêng HSV. Profiling: YOLO predict 26,85 ms (neural 19,78); embedding 24,07 ms (neural 16,94); DeepSORT update không gồm embedding 2,45 ms; HSV 0,717 ms/frame. Manager trên **1.270 frame đo còn active**: mean **0,605 ms**, p95 **0,874 ms**; neural cosine 0,071, HSV cosine 0,0077, overlap 0,0678, motion 0,193, scoring 0,0084 ms. Mean manager toàn video 0,252 ms bị kéo thấp bởi LOST. Thành phần neural nằm trong API tổng, không cộng lại. Chi phí từng phép được đo; không quy sự chậm so với baseline lịch sử cho guard khi chính inference đã tăng. Lượt component profiling có một lần test collection ngắn chạy đồng thời; hai plain controls không có tests đồng thời, dùng chúng để so FPS.

Có đúng **3.166 YOLO neural forward, 3.166 embedding forward và 3.166 crop call** trong lượt profiled; guard không thêm inference. [Timing](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/after.json), [plain control](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/after_control.json), [manifest](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/after_manifest.json). Production hashes không đổi suốt benchmark/diagnostic; bản profiler chạy benchmark giữ trong source snapshot, diagnostic bổ sung sau đó không thay guard.

**Kiểm chứng và file.** PASS: **127 tests**, build riêng `uav_perception` và import package build, `git diff --check`. Runtime acceptance INCONCLUSIVE, phase review CHANGES REQUIRED. Bản copy tests trong evidence từng gây pytest collect trùng; đã giữ nguyên nội dung và đổi hậu tố chính các bản copy thành `.snapshot`, rồi rerun toàn bộ: [log pass](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/tests_verified.log). Không sửa/xóa tests nguồn. [Acceptance](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/acceptance_v2.json) lưu SHA256 label/profile/raw geometry/evaluator.

File production và benchmark thật là luồng đang dùng; `tracking_identity_candidate.py`/`tracking_appearance.py` là thí nghiệm offline. Hai evaluator ở Git root là symlink alias, không phải hai codebase. Các `final*`/development runs là evidence từng phiên, không tự dọn. Snapshot working tree có 801 entries tại lúc chụp, gồm 23 tracked modifications và nhiều untracked; `git diff` không bao gồm guard/tests/evidence untracked. Các thay đổi control/simulation/docs sẵn có được bảo toàn. Không commit/reset, không xóa user files, không sửa `../software`.

**Phương án tối thiểu và thứ tự tiếp theo, chưa triển khai.** (1) Test deterministic bbox cùng người bị cắt chân nhưng head/top ổn định, sau đó trở lại full body. (2) Bảo vệ trusted motion history theo độ tin cậy geometry/extent, giữ history trước crossing khi bbox bị cắt; chỉ “torso không overlap” chưa đủ. Không nới vertical gate hoặc timeout toàn cục để che lỗi. Reuse history/embedding/Kalman hiện có; kiểm chứng lựa chọn extent/head anchor trước khi chốt spec. (3) Reproduce thật D3 dưới dạng development và regression B/C/D1/D2; ablation bật/tắt HSV. (4) Freeze source, gán nhãn/test event hoặc capture độc lập mới, benchmark cùng input, đối chiếu acceptance. Chưa có lý do thêm model/framework hoặc thay tracker.

Dependency/file ownership: một writer cho guard + node; labeler không đọc output guard; tests/evaluator có thể review riêng; benchmark chỉ chạy trên source đã freeze. Antigravity vẫn dừng. Nếu được giao lại sau này, những việc song song an toàn là raw-only annotation, reviewer evaluator/tests, và reviewer read-only code; không nhiều writer cùng checkout. Người dùng còn chốt coverage tối thiểu/thời gian abstain, FPS budget và nhãn/capture độc lập. Scope vẫn camera đứng yên; Phase 4B mới xét motion compensation. Following, 3D estimation, MotionArbiter và long-term recovery chưa triển khai.


Các lệnh chính đã chạy từ Git root (output dưới đây đã tồn tại; dùng tên mới nếu replay lại, không ghi đè evidence):

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
source ../software/venvs/yolo26/bin/activate
PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" python -m pytest -q
git diff --check
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" python evaluation/profile_phase4a_target_guard.py --max-frames 3166 --plain-control-frames 3166 --paired-appearance-ablation --output-dir evaluation/results/runs/phase4a_target_guard_20261013_final6
python evaluation/evaluate_phase4a_target_guard.py --annotations evaluation/annotations/phase4a_target_crossing_reviewed_20261013_v2.csv --detections evaluation/results/runs/tracking_3_people_20261009T084549Z/detections.csv --profile evaluation/results/runs/phase4a_target_guard_20261013_final6/after_profile_frames.csv --output evaluation/results/runs/phase4a_target_guard_20261013_final6/acceptance_v2.json
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" python evaluation/profile_phase4a_target_guard.py --max-frames 1322 --plain-control-frames 60 --diagnostic-range 1278 1321 --output-dir evaluation/results/runs/phase4a_final6_d3_neural_diagnosis_20261013
```
