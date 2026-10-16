# Review sau sửa lỗi — 06/10/2026

**Verdict: CHANGES REQUIRED.** Một số sửa lỗi có hiệu quả, nhưng chưa thể đóng audit hoặc coi Phase 4A đã được xác minh toàn luồng. Benchmark đang có thay đổi chưa nối đủ với perception; ba sửa perception nằm ở worktree riêng và chưa áp dụng vào source/installation của dự án chính.

## Phạm vi

- Đối chiếu toàn bộ 20 finding đánh số và các mục LOW/INFO, hygiene, kích thước file trong `project_audit_2026-10-06.md`.
- Kiểm tra working tree chính `/home/tungt/UAV_Project/project` và worktree `/home/tungt/UAV_Project/worktrees/perception-fresh-observation`, branch `codex/perception-fresh-observation`. Cả hai cùng HEAD `dfad9dd`; các sửa hiện tại chưa commit. Kết luận dựa trên nội dung working tree, không chỉ Git HEAD.
- Review thay đổi control/evaluation/perception, các đường launch/build/import, cấu hình GPU, tài liệu hiện tại và artifact GPU. Các asset/historical docs không đổi được đối chiếu theo audit cũ; XML/model/package được parse lại. Không kiểm toán từng file dependency ngoài `project/`.
- Không sửa source, merge, rebuild, dọn file hay chạy bay. Chỉ thêm báo cáo này. Runtime control dưới đây dùng ROS domain riêng và telemetry giả lập, không kết nối PX4.
- Áp dụng `skills/phase-review/SKILL.md`; phân biệt bằng chứng source, unit test, integration và runtime.

## Các finding còn phải xử lý trước

### HIGH — Perception fixes chưa có trong bản chạy chính

`src/uav_perception/uav_perception/yolo_detector.py:262,278,313,315` vẫn đưa RGB vào hai API BGR và lấy mọi confirmed track. `target_identity.py` chính vẫn có fallback chọn track đầu tiên. Installation chứa ba module yolo/identity/gimbal giống source chính; không có `image_utils.py`. `launch_phase4a_review.sh:74` build cây chính, rồi chạy module từ install đó.

Vì vậy một lần launch/build chính hiện vẫn chạy các lỗi audit #4–6. Đây là trạng thái tích hợp chưa hoàn thành, không phải chứng cứ sửa ở worktree không có tác dụng. Khi được yêu cầu tích hợp, cần mang đầy đủ module và test sang cây chính, build đúng cây và xác nhận module path khi chạy.

### HIGH — Raw detector/processing telemetry chưa có producer và bị báo có dữ liệu giả

`evaluation/benchmark_evaluator.py:97–104` subscribe `/yolo/raw_detections` và `/yolo/telemetry`; perception ở **cả hai cây** chỉ publish selected tracking detection và annotated image, chưa publish hai topic mới.

`benchmark_evaluator.py:442` luôn ghi `raw_detection_present=False` khi raw message chưa có. `metric_calculator.py:106` xác định telemetry available bằng việc có key trong sample, nên báo available ngay cả khi không nhận raw telemetry. Run thực tế có thể ghi raw detector miss cho mọi processed frame dù không hề đo detector. Duration telemetry vẫn thiếu và metric latency vẫn fallback về callback-arrival skew (`benchmark_evaluator.py:454`, `metric_calculator.py:140–144`). Auditor còn giữ cách đo cũ.

Khuyến nghị: hoàn thiện producer theo cùng source stamp, biểu diễn missing bằng unknown/availability riêng, không coi thiếu telemetry là detection miss; chỉ dùng duration đo trong node cho processing latency. Giữ arrival skew dưới tên riêng.

### HIGH — Matcher phụ thuộc thứ tự callback và xử lý late detection chưa đầy đủ

`benchmark_evaluator.py:329–334` finalize ngay khi có tracking detection. Raw/telemetry đến sau sẽ chỉ vào pending map, không cập nhật sample đã finalize. Các topic khác nhau không bảo đảm callback theo thứ tự mà test hiện đang đưa vào.

Tái hiện thứ tự tracking → finalize → raw → telemetry: sample vẫn `raw_detection_present=False`, `actual_processing_latency_ms=None` dù dữ liệu đã đến. Trường available vẫn là True.

`benchmark_evaluator.py:212–222` khi detection đến sau timeout chỉ đổi ba flag và counter, không điền track/bbox/confidence/raw/duration. Tái hiện detection có người: sample `is_processed=True`, `matching_failure=True` nhưng `track_present=False`, `track_id=None`, `detection_present=False`. Auditor có cùng kiểu cập nhật flag (`audit_camera_fps_accounting.py:127–132`). Điều này làm thấp retention và tạo mất track giả.

Khuyến nghị: ghép đủ thành phần theo source stamp, có trạng thái unresolved và chính sách deadline rõ ràng; late arrival phải cập nhật payload liên quan hoặc được loại khỏi các mẫu dùng đánh giá detector/tracker. Bổ sung test hoán vị callback và kiểm tra payload của late sample, không chỉ ba flag.

### HIGH — Identity gate mới vẫn có thể nhận nhầm người gần vị trí cũ

Worktree `src/uav_perception/uav_perception/target_identity.py:196–207` gate khoảng cách/IoU rồi chọn ứng viên gần nhất. Gate chặn được người ở xa, nhưng không chứng minh cùng người. Tái hiện: A ở `[0,0,100,100]`, A mất; B chiếm bbox đó và C ở `[10,0,110,100]`. B được gán logical identity của A và chuyển TRACKING, dù có hai ứng viên hợp lệ.

Khuyến nghị: từ chối reassignment mơ hồ; dùng bằng chứng appearance/association đủ tin cậy hoặc yêu cầu chọn target mới. Phải kiểm tra tình huống hai người đi qua nhau. Unit test hiện xác nhận spatial gate, chưa xác nhận identity của người thật.

### MEDIUM — Các lỗi control và metric cũ vẫn còn

- `altitude_control.cpp:59–64,100–105`: cập nhật rate chỉ đổi biến. Runtime executable hiện có chấp nhận 10 → 20 Hz nhưng cả hai cửa sổ 1,5 giây đều nhận 15 heartbeat. Cần cập nhật timer hoặc từ chối thay đổi runtime. Chuỗi arm vẫn phụ thuộc 10 tick (`:203`).
- `metric_calculator.py:189–194`: cùng ID ở mọi frame tại 0/0,4/0,8 giây vẫn cho retention 100% nhưng 2 losses và 2 reacquisitions. Cần phân biệt thiếu frame camera với mất target.
- `benchmark_evaluator.py:583,744` vẫn gọi unmatched là queue drop; auditor `:446,485` vẫn gọi subscriber count là số published. Không đủ chứng cứ kết luận source loss hay YOLO bỏ frame chỉ từ hai subscriber.
- `metric_calculator.py:82–90` trả schema thiếu `processed_frames` khi zero samples, nhưng summary `benchmark_evaluator.py:581,742` truy cập trực tiếp: vẫn có thể KeyError. Zero samples còn được ghi actual duration bằng requested duration và completion reason mặc định là reached.
- `metric_calculator.py:359–362` suy ra movement used for tracking từ drift. Lọc NaN và UNKNOWN đã tốt hơn, nhưng drift chỉ chứng minh chuyển động, không chứng minh ý định tracking.

### MEDIUM — Các giới hạn perception chưa xử lý

- Decoder chính bỏ `msg.step`; helper worktree `image_utils.py:7–17` vẫn chỉ nhận tightly packed. Ảnh rgb8 một pixel với step=4 tái hiện ValueError. Worktree bắt lỗi và bỏ frame thay vì crash, nhưng chưa hỗ trợ ảnh hợp lệ có padding.
- `enable_deepsort=False` vẫn không có selected target output, dù YOLO có detection.
- Lệnh gimbal ban đầu vẫn publish khi disable tracking; góc ban đầu chưa clamp.
- Bốn history list vẫn append không giới hạn và không có consumer.
- `gimbal_controller.py:102–111` vẫn tích lũy rad/frame, không dùng dt. Khi GPU tăng FPS, tốc độ gimbal theo wall time thay đổi; cần đo lại tuning/ổn định khi tích hợp GPU và perception.
- Worktree yêu cầu explicit reset sau LOST để nhận ID mới, nhưng node chưa expose reset/selection. Đó là chính sách an toàn hơn tự nhận người bất kỳ, song hiện phải khởi động lại node để chọn lại; cần interface nếu sản phẩm yêu cầu phục hồi mất target.

## Đối chiếu từng mục audit gốc

“Đã sửa” ở bảng dưới chỉ áp dụng cho hành vi được kiểm tra; không thay cho xác minh chuyến bay.

| # | Finding gốc | Trạng thái hiện tại |
|---|---|---|
| 1 | Lệnh README sai interpreter | **Đã sửa đường import**: dùng Python venv với `-m`; import thực tế PASS. Chưa launch model/simulator toàn luồng |
| 2 | Offboard không gate telemetry | **Sửa một phần tốt**: validity/finite, reference lock và freshness trước cất cánh đã có. Runtime không telemetry: 0 heartbeat; quá timeout: dừng phát trước cất cánh. Chưa xác minh PX4 state/ACK hoặc tình huống telemetry lỗi sau cất cánh |
| 3 | rate 0,5 gây SIGFPE | **Đã loại nguyên nhân crash cũ**: bỏ modulo ép integer; rate=0 exit 1 có log, rate=0,5 sống tới khi chủ động kết thúc. Chưa có giới hạn rate vận hành đầy đủ |
| 4 | Prediction cũ coi là observation | **Đã sửa trong worktree**, lọc `time_since_update == 0` ở node và manager, không refresh last_seen, clear bbox khi thiếu observation. **Main/install còn lỗi** |
| 5 | RGB/BGR | **Đã sửa trong worktree** cho tightly packed RGB/BGR/mono, dùng BGR ở model/tracker và RGB ở output. **Main/install còn lỗi** |
| 6 | Reassignment/reacquisition không an toàn | **Sửa một phần trong worktree**: chặn người xa và ID mới sau LOST; còn ambiguity gần/overlap, thiếu reset interface. Main giữ lỗi gốc |
| 7 | Detector metric đo tracker | **Chưa đóng**: evaluator đã có consumer raw, thiếu producer/availability; auditor và full-mode summary vẫn dùng semantics cũ |
| 8 | Latency không đo processing | **Chưa đóng**: consumer duration có test, chưa producer; fallback/skew và auditor còn đo cũ |
| 9 | Drop/source loss/matching | **Sửa một phần**: có drain và late flag; payload late/order callback/source accounting còn sai |
| 10 | Đổi rate không đổi timer | **Còn lỗi**, tái hiện trên executable hiện có |
| 11 | Ctrl+C nói sẽ land | **Còn lỗi**, `launch_simulation.sh:310`; cleanup vẫn chỉ kết thúc tiến trình |
| 12 | Disable DeepSORT mất output | **Còn lỗi** ở main và worktree |
| 13 | Bỏ row stride | **Còn lỗi hỗ trợ input**; worktree chỉ cải thiện bắt exception |
| 14 | Disable gimbal vẫn publish initial | **Còn lỗi** ở main và worktree |
| 15 | History không giới hạn/không dùng | **Còn lỗi** ở main và worktree |
| 16 | Loss phụ thuộc FPS | **Còn lỗi**, tái hiện 2 false losses/100% retention |
| 17 | Convergence thiếu hold window | **Đã sửa ở main**, yêu cầu đủ window, có regression test PASS |
| 18 | Missing/NaN telemetry | **Sửa tốt phần crash/lọc NaN/UNKNOWN**; còn inference “movement used for tracking” từ drift, chưa validate mọi giá trị finite |
| 19 | Run thiếu duration ghi đủ | **Sửa một phần**: có requested/actual span/completion reason cho verify; zero/one sample còn fallback requested, full mode chưa phản ánh mọi lý do dừng |
| 20 | Gimbal rad/frame | **Còn giới hạn thiết kế**, GPU làm cần kiểm chứng lại hành vi |

## GPU, kiến trúc, hygiene và tài liệu

- GPU configuration có phạm vi gọn: hai Gazebo helper source `config/sim/gazebo_rendering.sh`; giữ env override và opt-out. Artifact cũ ghi renderer D3D12 RTX 4060, server 30,08 wall FPS, GUI 29,42, ROS subscriber best-effort 20,17 và reliable 28,08. Số liệu nhất quán với tài liệu, **không phải full-stack FPS**. Không cần chạy lại phép đo GPU không có thay đổi mới trong lượt review này.
- Dự án vẫn ở mô phỏng, hover và gimbal visual servoing; estimator, TF/feedback góc thật, MotionArbiter/body following vẫn chưa triển khai. Chưa thấy việc thêm kiến trúc quá phạm vi; cần đóng correctness/integration trước khi mở rộng.
- `benchmark_evaluator.py` tăng 704 → **808 dòng**, auditor 504 → **523**, metric 356 → **437**. Perception main vẫn 479 dòng; worktree đã tách image helper nhưng frame pipeline vẫn lớn. Ưu tiên pure stamp matcher/metric helpers sau khi chốt semantics; chưa cần thêm ROS package/framework.
- Các ứng viên dư ở audit cũ còn tồn tại: literal `~/.cache/ultralytics/yolo26n.pt` (~5,3 MiB), texture materials trùng meshes, texture `*-unmodified.png`. Chưa có yêu cầu xóa, nên giữ nguyên. Root evaluator/metric là symlink, không phải code copy.
- Logical target ID vẫn không được publish riêng, `class_id` còn chứa track ID. Timestamp zero được sửa trong manager worktree; main còn sentinel cũ.
- Contract camera vẫn khẳng định publisher Best Effort/depth 1 trong khi bridge YAML chưa explicit cấu hình; phải phân biệt subscriber QoS với publisher QoS. Metadata control vẫn thiếu direct dependency `rcl_interfaces`.
- PROGRESS còn ghi evaluation 7+3 tests; hiện là **11 metric + 6 matching**. Main perception 14 tests, worktree 23 tests. Không nên thay tài liệu main thành 23 trước khi tích hợp worktree.
- Toàn bộ thay đổi liên quan vẫn chưa commit; tên branch riêng không đồng nghĩa bản sửa đã có trong main hoặc đã được lưu trong commit.

## Kiểm chứng lượt review này

| Kiểm tra | Kết quả |
|---|---|
| Main perception, đặt PYTHONPATH đúng package | **14/14 PASS** |
| Worktree perception, đặt PYTHONPATH đúng package | **23/23 PASS** |
| Main evaluation với ROS/px4_msgs environment | **17/17 PASS** (11 metric, 6 matching) |
| README venv import torch/ultralytics/DeepSORT/node | **PASS**, interpreter đúng venv |
| Runtime control, ROS domain 199, telemetry giả lập | Gate trước cất cánh **PASS**; dynamic timer **FAIL**: 15 bản tin/1,5s cả trước và sau 10→20 |
| Runtime startup rate=0 / rate=0,5 | 0 bị từ chối rõ ràng; 0,5 không SIGFPE trong cửa sổ kiểm tra, chưa kiểm tra heartbeat dưới telemetry với rate này |
| Tái hiện matcher/raw callback/late payload | **Xác nhận các lỗi HIGH** nêu trên |
| Tái hiện camera chậm/empty metric/ambiguous identity/padded image | **Xác nhận các lỗi còn lại** nêu trên |
| Python AST parse, XML parse 13 file, Bash syntax từng script, ShellCheck warning | **PASS** |
| git diff --check main và worktree | **PASS** tại thời điểm review |
| Main source so với installed perception | Ba module bằng nhau; chưa có image_utils ở main/install |
| Gazebo + PX4 + CUDA inference + perception fixes end-to-end | **CHƯA CHẠY**; không kết luận ổn định bay hay target identity bằng unit tests |

Lần gọi pytest worktree đầu tiên không đặt PYTHONPATH nên collection lỗi import; đã chạy lại bằng đường package đúng và có 23 PASS. Lần đo control đầu tiên dùng subscriber durability không tương thích nên không nhận heartbeat; đã sửa cấu hình subscriber Volatile trước khi lấy kết quả rate trong bảng. Không dùng hai lần đo sai cấu hình đó làm bằng chứng hành vi.

## Thứ tự hoàn thiện đề xuất

1. Tích hợp perception worktree vào đúng cây sẽ launch; hoàn thiện chính sách ambiguity/selection và xác minh bằng node-level test + tình huống hai người, mất detection, ID đổi.
2. Nối producer raw/duration và sửa matcher cho thứ tự callback/late payload/unknown; đóng metric camera gap và empty samples trước khi chạy benchmark mới.
3. Sửa dynamic timer, thông báo cleanup, các flag/input/history; xác minh control với telemetry invalid/stale và PX4 state/ACK theo phạm vi được giao.
4. Chạy Phase 4A full-stack trên GPU và đo cả render, ROS receive, inference, actuation. Chỉ sau đó mới đóng milestone và dùng số liệu để phát triển body following.
