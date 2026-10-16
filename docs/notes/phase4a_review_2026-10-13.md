# Phase 4A — review 2026-10-13

**CHANGES REQUIRED. Chưa đủ điều kiện đóng Phase 4A.** Bản guard trước (`final4`) xác nhận nhầm người; bản mới đang trong working tree (`final5`) từ chối cả association hợp lệ và khóa LOST sớm. Antigravity đã dừng. Không commit, reset hoặc xóa file; không triển khai Phase 4B, following, 3D estimation hay MotionArbiter.

**Pipeline đã xác minh**

`Image → YOLO26n → DeepSORT → TargetIdentityManager → GimbalController`.

- YOLO xuất mọi detection person; DeepSORT crop ảnh BGR và gọi MobileNet embedder qua `generate_embeds → crop_bb → embedder.predict`.
- Manager reuse `Track.get_feature()`; lấy bbox detection bằng `to_ltrb(orig=True, orig_strict=True)`. Không crop lại hoặc chạy ReID mới trong manager. Bộ PID dùng embedding + histogram màu trong `evaluation/tracking_identity_candidate.py` là thí nghiệm riêng, không phải guard production.
- DeepSORT đã có Kalman mean/covariance với state `(x,y,a,h,vx,vy,va,vh)`, track age, confirmed state và `time_since_update`. Kalman velocity tính theo bước frame; không được dùng trực tiếp như px/s. Guard dùng history bbox và `Image.header.stamp` để tính motion theo thời gian nguồn.
- Node chỉ đưa track confirmed/fresh vào manager. Bbox được manager xác nhận mới tới gimbal và target detection topic; UNCERTAIN/PREDICTING/LOST không xuất bbox. Gimbal giữ lệnh cuối khi không có observation.
- Logical target ID nội bộ luôn là `1`; Detection2D ID hiện xuất raw tracker ID. Hai loại ID này không chứng minh người thật vẫn là target.

**Nguyên nhân và regression có bằng chứng**

1. **BLOCKER — confirmed target chuyển người.** Trong `final4`, frame 1118 đổi association TID 20 → 38. Incumbent similarity 0,863; candidate mới 0,891, chênh 0,028. Điểm motion từ history cũ làm candidate mới thắng điểm tổng dù appearance chưa phân biệt đủ. Frame 1120 xác nhận người áo xanh dương, còn target áo đỏ ở bên trái. D2 có **10/14 frame rõ xác nhận sai**, không phải 12 sau khi sửa nhãn. [Ảnh đối chiếu](../../evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/review_before_after_20261013.png).
2. **BLOCKER — fix mới làm mất association hợp lệ.** Veto incumbent mới tại `target_identity.py` chặn lần đổi TID hợp lệ từ frame 523; observation cuối được chấp nhận là frame 510. Frame 554 (69,796 s nguồn) chuyển LOST dù target vẫn trong ảnh; trạng thái terminal ngăn mọi xác nhận sau đó. Toàn capture: TRACKING 509, UNCERTAIN 43, LOST 2.613, SEARCHING 1.
3. **HIGH — motion history chưa được bảo vệ đầy đủ khi overlap.** `clean_crop` hiện chỉ bảo vệ gallery; `_accept()` vẫn ghi bbox bẩn vào trusted motion history. Test `test_dirty_crop_can_confirm_after_two_consistent_observations_without_gallery_update` còn cho phép xác nhận crop giao mạnh, chưa đáp ứng yêu cầu đánh dấu crossing mạnh là uncertain. Đây là vấn đề cần sửa, chưa được coi là nguyên nhân duy nhất của mọi switch.

**Dataset và acceptance**

Capture gốc có 3.166 frame. B/C/D1/D2 là development vì đã mở output để chẩn đoán; D3 frame 1268–1302, lấy mỗi hai frame, là event-held-out cho bản `final5`. D3 gồm 12 frame target rõ và 6 frame ambiguous. Tất cả vẫn cùng một capture, chưa có capture độc lập hoặc người thứ hai adjudicate nhãn.

Nhãn được gán từ video thô + bbox YOLO ẩn danh, không dùng TID/overlay/proxy màu để chọn detection. Rà lại đã sửa detection index ở D2 frame 1126, 1134, 1184, 1186 thành D0; bản nhãn trước được giữ trong `evaluation/annotations/revisions/`.

| Kết quả | final4 | final5 hiện tại |
|---|---:|---:|
| Đúng target trên 89 frame rõ được gán nhãn | 71 | 6 |
| Xác nhận sai bbox target | 10 | 0 |
| Abstain trên frame rõ | 8 | 83 |
| Xác nhận trên 61 frame ambiguous, chưa kiểm chứng identity | 39 | 0 |
| Holdout D3: đúng target / frame rõ | 12/12 | 0/12 |
| Raw TID reassociations được manager chấp nhận, toàn capture | 49 | 0 |
| Gate | FAIL | INCONCLUSIVE |

Không báo “0 logical switches” chỉ vì chuỗi ID vẫn là `1`. Đếm sự kiện đổi người trên toàn video chưa có đủ GT; báo mismatch frame và gimbal confirmation riêng. `final5` không có mismatch quan sát được nhưng không có xác nhận nào ở holdout, nên không được PASS.

8,12% PID trống trong báo cáo 2026-10-09 là 667/8.212 detection phân loại được của thí nghiệm gallery: 178 chưa có TID confirmed và 489 có TID nhưng PID từ chối. Đây không phải tỷ lệ YOLO miss, không phải target-specific error và không chứng minh production guard đã đúng.

**Performance đã đo**

Cùng MP4, 3.166 frame, bỏ 50 frame warmup. Chạy YOLO/DeepSORT thật và ROS publications không có subscriber; không phải camera/DDS FPS hoặc gimbal di chuyển thật.

| Lượt | Mean process frame | Replay FPS |
|---|---:|---:|
| Baseline không instrument | 24,77 ms | 40,37 |
| final5 plain control | 28,79 ms | 34,73 |
| final5 component profiling | 30,70 ms | 32,57 |

final5: YOLO predict 13,15 ms (neural forward 9,20 ms); DeepSORT update 12,69 ms, gồm embedding 11,25 ms (neural forward 7,51 ms) và phần còn lại 1,44 ms. Các số neural là thành phần nằm trong tổng, không cộng lần nữa.

Manager mean **0,159 ms trên 503 frame đo còn active/uncertain**, p95 0,238 ms; mean toàn capture 0,030 ms bị kéo thấp bởi LOST. Khi còn active: cosine 0,020 ms; overlap 0,012 ms; motion 0,019 ms; scoring 0,006 ms. Có đúng 3.166 YOLO forward và 3.166 batch embedding forward; guard không thêm neural inference. Chênh FPS giữa các lượt chưa tách ảnh hưởng GPU/load và instrumentation, không quy toàn bộ cho guard.

**File và kiểm chứng**

- Production candidate: `src/uav_perception/uav_perception/target_identity.py`, `yolo_detector.py`; giữ DeepSORT và interface gimbal hiện có.
- Tests/evaluation: `src/uav_perception/test/`, `evaluation/annotate_phase4a_target_events.py`, `evaluate_phase4a_target_guard.py`, `profile_phase4a_target_guard.py`, `test_phase4a_target_guard_evaluator.py`, annotations và run artifacts. Evaluator lưu SHA256 của input, không suy identity từ constant logical ID; test ngăn always-abstain hoặc sai người được PASS.
- **PASS:** toàn bộ 103 tests, 3 dependency deprecation warnings; `git diff --check`; source hashes của `final5` khớp production hiện tại.
- **NOT ACCEPTED:** crossing runtime; holdout target continuity; full simulation của guard mới. Không chạy lại Gazebo/PX4 khi MP4 replay đã đủ bác bỏ guard.
- Git có nhiều thay đổi sẵn có; 663 file untracked tại thời điểm review trước khi thêm báo cáo này, gồm guard, tests, notes và phần lớn evidence. `git diff` đơn thuần không chứa các file đó. Các symlink evaluator ở Git root là alias tới file trong `evaluation/`, không phải hai bản code. Các run `final*` là evidence có chủ ý; không dọn chúng. User changes và báo cáo lịch sử được giữ nguyên.

Lệnh verification đã chạy từ Git root, với ROS Jazzy, workspace dependencies và venv YOLO được source trước:

```bash
PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" python -m pytest -q
git diff --check
python evaluation/profile_phase4a_target_guard.py --output-dir evaluation/results/runs/phase4a_target_guard_20261012_codex_final5 --max-frames 3166 --plain-control-frames 3166
python evaluation/evaluate_phase4a_target_guard.py --annotations evaluation/annotations/phase4a_target_crossing_reviewed_20261013.csv --profile evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/after_profile_frames.csv --output evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/acceptance_reviewed_20261013.json
```

Profiler từ chối ghi đè evidence: dùng output directory mới khi chạy lại.

**Bước tiếp theo đề xuất, chưa triển khai**

1. Tái hiện riêng B frame 505–554 và D1/D2 1080–1186; giữ D3 làm test, không chỉnh ngưỡng theo D3.
2. Đổi veto incumbent tuyệt đối thành ambiguity rule có giới hạn; bảo vệ cả motion history và appearance trước crossing. Dùng history sạch hiện có, velocity theo source timestamp, motion gate theo dt/scale và appearance margin tách biệt. Không ép chọn người khi embedding không phân biệt được; không tăng timeout để che lỗi association.
3. Test phải cho phép target đổi TID hợp lệ khi TID cũ đã thuộc người khác, đồng thời từ chối người khác thắng nhờ stale motion. Review lại test dirty crop theo yêu cầu crossing uncertain.
4. Benchmark lại cùng input, rồi nghiệm thu **0 confirmed wrong-person associations**, báo coverage/abstain và xác nhận đúng target cả trước lẫn sau crossing. Thiếu nhãn rõ hoặc toàn abstain là INCONCLUSIVE.

Dependency: annotation phải độc lập với guard output; guard spec trước tests/replay; chỉ một writer production. Những việc có thể tách song song sau khi được giao lại: adjudicate/capture test độc lập, review evaluator, và profiling với output directory riêng. Antigravity hiện vẫn dừng.

Cần chốt thêm coverage tối thiểu, thời gian uncertain tối đa cho crossing ngắn, và nhãn/capture độc lập trước khi đóng phase. Phase 4B mới xử lý camera/gimbal motion compensation; following, 3D estimation và MotionArbiter vẫn chưa triển khai.

Evidence: [final4 acceptance đã sửa nhãn](../../evaluation/results/runs/phase4a_target_guard_20261012_codex_final4/acceptance_reviewed_20261013.json), [final5 acceptance](../../evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/acceptance_reviewed_20261013.json), [final5 profile](../../evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/after.json), [source manifest](../../evaluation/results/runs/phase4a_target_guard_20261012_codex_final5/after_manifest.json), [nhãn đã review](../../evaluation/annotations/phase4a_target_crossing_reviewed_20261013.json).
