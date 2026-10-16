# Phase 4A — báo cáo cuối, 2026-10-13

**CHANGES REQUIRED; chưa đóng Phase 4A.** Antigravity đã dừng. Bản `final6` không xác nhận sai người trong các cửa sổ đã gán nhãn, nhưng chưa chứng minh continuity sau D3. Báo cáo này sẵn để người dùng tự review; review độc lập chưa thực hiện.

**Pipeline đã xác minh:** `YOLO26n → DeepSORT → TargetIdentityManager → GimbalController`. Manager reuse embedding MobileNet 1.280 chiều qua `Track.get_feature()`; dùng bbox detection gốc, confirmed/fresh. Kalman velocity theo frame chỉ reuse hướng; velocity guard dùng history và `Image.header.stamp`. Gimbal/target topic chỉ nhận bbox manager xác nhận; không nhận bbox dự đoán khi abstain.

**Nguyên nhân có bằng chứng:** bbox target bị cắt ngắn ở frame 1283–1284 nhưng vẫn vào motion history. Centroid đổi do bbox, tạo velocity dọc giả **−58 px/s**. Frame 1292 có đúng target, neural cosine **0,832**, HSV **0,967**, nhưng vertical residual **83,87 > gate 38,71 px**, nên bị reject. Sau đó terminal LOST tại frame 1321 dù người vẫn trong ảnh. Đây là lỗi history/geometry; không phải bằng chứng cần thay tracker. [Ảnh đối chiếu](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/D3_failure_review.png).

**Thay đổi đã kiểm tra:** guard có anchor/reference giới hạn, motion theo source time, validation cho cả TID cũ/mới, freeze reference khi crop bẩn, chặn transition đông người khi detection giảm, hai observation mới để xác nhận lại. Histogram HSV torso lấy từ crop DeepSORT đã có; crop và embedding batch chỉ chạy một lần. Không thêm model/dependency hoặc following/3D/MotionArbiter.

**Acceptance:** nhãn target-only v2 gồm **215 frame liên tục**, 176 rõ và 39 ambiguous. B/C/D1/D2 là development; D3 là event-held-out trong cùng capture, không phải capture độc lập. Nhãn được rà từ ảnh thô/YOLO index ẩn danh; không dùng TID hay output guard.

| Kết quả | final6 |
|---|---:|
| Xác nhận sai target/gimbal trong cửa sổ nhãn | **0** |
| Coverage trên frame rõ | **127/176 = 72,16%** |
| Abstain trong cửa sổ nhãn | **88/215** |
| Xác nhận trên frame ambiguous | **0** |
| Development: đúng trước và sau crossing | **4/4 event** |
| D3: đúng trước / sau | **11 / 0** |
| Raw target-TID reassociations toàn capture | **7**, được phép |
| Gate | **INCONCLUSIVE** |

Abstain không tính là target switch. Gate còn yêu cầu đúng target trước **và** sau mỗi crossing, so bbox với detection cạnh tranh; không PASS nhờ always-abstain hoặc logical ID luôn là `1`. Toàn capture có 1.846 frame LOST sau failure; chưa có GT để tuyên bố 0 physical switches trên cả video. 8,12% PID thiếu trước đây là PID refusal/unconfirmed TID của thí nghiệm offline, không phải target-specific error hay YOLO miss.

**Benchmark thật**, cùng 3.166 frame, bỏ 50 warmup; camera giữ nguyên trong replay, ROS domain riêng, không điều khiển flight/gimbal thật:

| Lượt | Mean frame | Replay FPS |
|---|---:|---:|
| Baseline lịch sử | 24,77 ms | 40,37 |
| Cùng process hiện tại, HSV tắt | 59,92 ms | 16,69 |
| final6, HSV bật, không instrument | 60,29 ms | 16,59 |
| final6, component profiling | 61,04 ms | 16,38 |

Guard mean **0,605 ms**, p95 **0,874 ms** trên 1.270 frame đo còn active; HSV **0,717 ms/frame**. Đúng 3.166 YOLO forward, 3.166 embedding forward và 3.166 crop call: không thêm inference. Chênh plain bật/tắt +0,38 ms là tổng đo được; chưa quy mức giảm so với baseline lịch sử cho guard. HSV tăng coverage từ 89 lên 127 frame đúng và giúp xác nhận sau C/D2, nhưng chưa giải quyết D3.

**Verification/file:** **127 tests PASS**, build/import `uav_perception` PASS, `git diff --check` PASS; runtime acceptance chưa đạt. Production hashes không đổi suốt benchmark/diagnostic. Core thay đổi ở `target_identity.py`, `yolo_detector.py`, `image_utils.py`; tests/evaluator/profiler và annotations tạo bằng chứng. Hai evaluator ở root là symlink alias. Guard/tests/evidence có nhiều file untracked: `git diff` không phản ánh hết; user changes, cache và mọi run cũ được giữ, không commit/reset/xóa.

**Phương án tiếp theo, chưa triển khai:** test bbox cùng người bị cắt rồi trở lại full body; bảo vệ trusted motion history theo geometry/extent thay vì chỉ torso overlap. Giữ DeepSORT, embeddings, history và Kalman hiện có. Không tăng timeout/nới toàn bộ gate để che lỗi. Reproduce D3 như development, freeze source rồi kiểm tra event/capture mới chưa dùng chỉnh code; benchmark và review lại. D3 đã mở để chẩn đoán nên không dùng làm holdout cho vòng sửa theo D3.

Dependency: một writer guard/node; labeler raw-only và reviewer evaluator có thể làm song song khi được giao lại, profiling sau source freeze. Antigravity vẫn dừng. Còn cần chốt coverage/abstain, FPS budget và nhãn/capture độc lập. Phase 4B mới xử lý camera motion; long-term recovery vẫn ngoài phạm vi.

[Acceptance và SHA256](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/acceptance_v2.json) · [Bằng chứng chi tiết, timing từng phép, lệnh và file](phase4a_final_review_evidence_2026-10-13.md).
