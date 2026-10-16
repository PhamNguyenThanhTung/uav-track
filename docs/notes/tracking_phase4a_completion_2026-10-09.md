# Hoàn tất kiểm thử DeepSORT — Phase 4A

**Kết luận: lượt chạy cuối đã hoàn tất; DeepSORT chưa đủ điều kiện nghiệm thu.** TID vẫn đổi và chuyển sang người khác trong giao cắt. PID cải tiến duy trì tốt hơn và chính sách ra/vào khung hình PASS, nhưng nghiệm thu toàn bộ PID còn **INCONCLUSIVE**: có những lúc không gán ID, nhãn chấm theo màu bị nhiễm khi che khuất và chưa có annotation độc lập cho toàn bộ video.

Không commit/push. Không triển khai UAV following, 3D estimator hoặc MotionArbiter. Các thay đổi production đang có trong working tree được giữ nguyên.

## 1. Bằng chứng cuối đã hoàn tất

Thư mục bằng chứng: [`tracking_3_people_20261009T084549Z`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z).

- Pipeline thật trong test: camera Gazebo → YOLO26n CUDA → DeepSORT → PID. Observer nhận `/custom_x500/camera/image_raw`, dùng YOLO26n `imgsz=640`, confidence=0,22 và lớp person. Tracker/PID không nhận nhãn actor. Nhãn màu chỉ dùng cho scoring.
- 3.166 frame, timestamp mô phỏng 36,964–234,204 s; chấm so sánh từ 50 s. Dữ liệu gốc: 8.871 detection; cửa sổ chấm có 8.212 detection phân loại được và 29 detection chưa phân loại được.
- `observer.log` ghi đạt mốc 234,204 s và lưu kết quả. `cleanup_status.txt`: `landed=true`, `observer_exit=0`. `altitude_control.log` ghi descent từ 2,91 m xuống 0,03 m. Không còn tiến trình test Gazebo/PX4/observer khi rà soát.
- Ba video đều có đủ 3.166 frame, giải mã được frame cuối. Cache hash và hash nguồn replay khớp; mọi PID/TID/row trong replay guarded khớp lượt chạy trực tiếp. Timestamp tăng liên tục, số frame liên tục, không có một ID gán cho hai detection cùng frame.
- Do bằng chứng đầy đủ và tracker không được sửa trong lượt rà soát này, không khởi động lại Gazebo hoặc làm lại capture/replay đã hoàn tất.

Các kiểm tra máy đọc nằm trong [`completion_audit/final_metrics.json`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/final_metrics.json). Báo cáo cũ, cache, summary và video gốc giữ nguyên.

## 2. Camera và kịch bản

UAV test hover 3 m, giữ vị trí; gimbal cố định yaw=-1,5708, pitch=0,82 rad. Camera 1280×720, horizontal FOV=90°. A/B đi ngược chiều và quay lại, C đi qua vùng giao nhau, sau đó có hai lần ra khỏi ảnh có chủ ý. World reentry có quỹ đạo kéo dài; dừng tại 234 s trước mốc loop sớm nhất 236 s.

Giai đoạn A có 205/205 frame thấy đủ ba người, YOLO nhận đủ ba và PID đủ ba. B/C/D có nhiều lần che khuất, YOLO không luôn tách đủ người. Screenshot và video xác nhận ba người toàn thân khi tách biệt. Không coi thiếu detection do che khuất là một lần đổi ID.

## 3. So sánh trên cùng dữ liệu

TID là ID track của DeepSORT; PID là danh tính tạm trong phiên do gallery cấp động. Không gán cố định 1/2/3 theo actor.

| Chỉ số | Baseline TID | Guarded TID | Guarded PID |
|---|---:|---:|---:|
| Đổi ID heuristic trong khoảng liên tục ≤1 s | 278 | 162 | 0 |
| Đổi ID ở các quan sát thấy rõ, kể cả sau mất dài | 64 | 23 | 1 |
| Tổng ID dư theo actor, N−1 | 115 | 48 | 2 |
| Detection có ID | 98,32% | 97,83% | 91,88% |
| Detection có ID và đúng nhãn neo, proxy | 52,78% | 88,04% | 91,87% |
| Đúng nhãn trong các assignment đã có neo, proxy | 58,04% | 94,07% | 99,987% |
| Unknown ID trên detection phân loại được | 138 / 1,68% | 178 / 2,17% | 667 / 8,12% |
| Assignment chưa có neo rõ | 607 | 348 | 0 |
| Assignment khác nhãn neo, proxy | 3.133 | 456 | 1 |
| Trong đó khác nhãn neo khi thấy rõ | 1.081 | 110 | 0 |

Định nghĩa để đọc bảng đúng:

- Neo danh tính của mỗi ID lấy từ **quan sát thấy rõ đầu tiên**, kể cả warmup, rồi giữ nguyên. Không dùng majority vote tương lai để sửa nhãn. “Đúng” nghĩa là nhãn áo hiện tại bằng nhãn neo; đây không phải tỷ lệ danh tính thật được annotation độc lập.
- Detection phân loại được: nhãn áo A/B/C chiếm ≥0,48. Thấy rõ: fraction ≥0,9 và overlap bbox ≤0,1. Assignment chưa có neo không được đưa vào mẫu số đúng/sai có neo.
- Coverage đúng dùng mẫu số toàn bộ 8.212 detection phân loại được; accuracy có neo dùng mẫu số correct + false. Detection không có ID gồm track chưa xác nhận/mất association và PID từ chối. Với PID: 178 detection chưa có TID được xác nhận, thêm 489 có TID nhưng PID để trống.
- 29 detection chưa có nhãn GT đáng tin được báo riêng, không tính đúng/sai. Unknown ID khác unknown ground-truth label và khác detection loss.
- “ID dư” là tổng N−1 theo actor, **không phải FRAG chuẩn MOT**. Nó chịu ảnh hưởng ID switch, nhãn crop bị nhiễm và reentry cấp ID mới có chủ ý. Hai ID dư của PID gồm một nhãn B bị nhiễm bởi PID 3 và lần C nhận PID 4 sau hết hạn; ở clear-view chỉ còn một lần có chủ ý.
- Đây là phép đo proxy, không phải IDF1/HOTA/IDSW chuẩn MOTChallenge. Descriptor cải tiến và nhãn chấm đều dùng màu áo, nên kết quả thuận lợi trên ba áo khác màu. **99,987% không chứng minh độ chính xác trên người thật.**

Phép đối chứng chỉ tăng `max_age` từ 30 lên 90, giữ n_init=1: 313 lần đổi TID, 53 ID dư, 99,37% có ID nhưng chỉ 37,87% coverage đúng proxy. Tăng tuổi track đơn thuần làm giữ nhiều association sai lâu hơn trong dữ liệu này.

## 4. Kết quả từng giai đoạn

Đếm trong từng cửa sổ, không tính chuyển đổi băng qua ranh giới giai đoạn. PID không đổi trong các giai đoạn B/C/D, nhưng vẫn có khoảng không xuất PID.

| Test | Frame | Đổi TID baseline → guarded | Đổi PID | PID unknown trên detection phân loại | Đánh giá |
|---|---:|---:|---:|---:|---|
| A — tách biệt, 50–62 s | 205 | 0 → 0 | 0 | 0% | PASS cho kịch bản tách biệt |
| B — giao cắt, 66–82 s | 255 | 37 → 21 | 0 | 11,51% | TID FAIL; PID continuity PASS theo proxy, nghiệm thu INCONCLUSIVE |
| C — giao cắt ba người, 82–102 s | 316 | 39 → 19 | 0 | 10,41% | TID FAIL; PID continuity PASS theo proxy, nghiệm thu INCONCLUSIVE |
| D — lặp giao cắt, 102–162 s | 965 | 116 → 69 | 0 | 8,64% | TID FAIL; PID continuity PASS theo proxy, nghiệm thu INCONCLUSIVE |
| E — ra/vào ngắn, 162–188 s | 397 | 31 → 20 | 0 | 7,51% | Chính sách reentry PASS; có cờ nhãn bị nhiễm |
| F — ra/vào dài, 188–234,204 s | 745 | 53 → 33 | 0 theo gap≤1 s | 8,78% | Chính sách hết hạn PASS; một clear-view đổi PID có chủ ý |

YOLO detected actor-frame / tổng visible actor-frame theo proxy: A 615/615; B 731/764; C 889/938; D 2.714/2.868. Đây là tổng nhãn theo frame, không phải detection recall từ bbox GT chuẩn; khi che nhau nhãn pixel và crop có thể khác nhau.

## 5. Chính sách ID đã kiểm chứng

Gallery giữ 15 giây **thời gian mô phỏng** từ quan sát được chấp nhận, không phải 15 frame. `max_age` của DeepSORT tính bằng frame. Không lưu gallery qua restart; ID monotonic trong phiên, không tái dùng.

| Tình huống | Vắng khỏi ảnh | Khoảng giữa hai quan sát được chấp nhận | PID | TID | Kết quả |
|---|---:|---:|---|---|---|
| Ngắn | 8,316 s | 9,604 s <15 s | 3 → 3 | 90 → 106 | PASS |
| Dài | 27,128 s | 28,544 s >15 s | 3 → 4 | 106 → 129 | PASS |

Kiểm tra lại bằng toàn bộ track rows, không chỉ crop fraction≥0,9: cả hai khoảng vắng có 0 quan sát C và 0 quan sát của PID cũ. Kết quả và ảnh bổ sung trong [`completion_audit/reentry`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/reentry).

Ngắn hạn ưu tiên nối PID nếu ngoại hình/motion đủ tin cậy và ứng viên duy nhất; khi cạnh tranh/không sạch thì có thể để trống. Hết hạn cấp PID mới; không ép mọi reentry quay lại ID cũ. Unit tests kiểm tra cạnh tranh một-một, hai người giống nhau, crop sát biên, clock reset và ngưỡng ngoại hình ở bước IoU.

## 6. Lỗi xác minh và sửa trong lượt hoàn tất

**Lỗi log loss:** observer cũ đóng `events[-1]` khi một actor phục hồi. Nếu actor A và B cùng mất dấu, sự kiện cuối có thể thuộc B dù A vừa trở lại. Ví dụ tracking loss A bắt đầu 36,964 s thực tế kết thúc 37,128 s nhưng summary cũ không đóng đúng sự kiện.

Sửa tối thiểu: đóng sự kiện còn mở của đúng actor bằng `tracking_loss_intervals.close_loss_interval`, dùng cho cả detection và tracking loss. Thêm regression mất dấu đồng thời. Không đổi detection, association, PID, gimbal hoặc controller. Tái hiện bằng `frames.csv` thật: dựng lại 156 detection-loss và 196 tracking-loss, tất cả đóng đúng actor. Tổng duration cộng theo actor lần lượt 28,680 s và 29,312 s; dài nhất 0,628 s và 0,924 s. Không dùng tổng này như thời gian mất chung của toàn cảnh. Summary lịch sử giữ nguyên; đọc `corrected_live_loss_intervals` trong audit mới.

**Lỗi bộ kiểm tra reentry:** trước đây “0 quan sát khi vắng” chỉ đếm crop nhãn C có fraction≥0,9; có thể bỏ sót assignment bẩn. Nay kiểm tra cả tất cả crop nhãn C, toàn bộ PID cũ và khoảng cách thời gian so với TTL. Chạy lại trên bằng chứng thật, cả hai trường hợp vẫn PASS.

## 7. Lỗi còn lại và kết luận nghiệm thu

- **HIGH — TID đổi người và phân mảnh khi giao cắt.** Bằng chứng baseline tại 68,312 s: TID 1 neo A ban đầu nhưng bbox bao người áo xanh lá B, trong khi fraction=1 và overlap=0,0228. Bản guarded TID vẫn có trường hợp tại 96,000 s. Ảnh [`baseline_clear_mismatch.png`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/baseline_clear_mismatch.png) và [`guarded_clear_mismatch.png`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/guarded_clear_mismatch.png). Tăng `max_age` chưa giải quyết association sai.
- **MEDIUM — PID chưa đủ bằng chứng độc lập.** Có 1 assignment trái nhãn neo tại 187,740 s; hai cờ chuyển nhãn C→B→C tại 187,740/188,036 s. Kiểm tra ảnh cho thấy PID 3 bao người áo xanh dương, crop có áo xanh lá phía trước. Có thể là label contamination; chưa xác nhận đây là một lần gán sai danh tính thật. Giữ nguyên cờ và timestamp, không che bằng con số 0 switch.
- **MEDIUM — PID có khoảng unknown.** 667/8.212 detection không có PID; khi thấy rõ 39/2.555 không có PID (1,53%). Cần xác định ngưỡng coverage chấp nhận và annotation độc lập trước khi kết luận PASS toàn bộ.
- **NOT RUN:** người mặc giống nhau ngoài unit test, thay đổi ánh sáng mạnh, camera chuyển động mạnh, video người thật, lưu PID xuyên phiên. Không suy rộng kết quả mô phỏng sang các trường hợp này.

**Verdict phase review: CHANGES REQUIRED để nghiệm thu DeepSORT.** Kiểm thử đã hoàn tất; kết quả test không đạt khác với test chưa chạy. Raw DeepSORT FAIL tiêu chí IDSW=0 trong giao cắt. PID đạt chính sách memory/reentry và ổn định hơn, nhưng nghiệm thu danh tính toàn bộ còn INCONCLUSIVE. Bản guarded vẫn là test opt-in, chưa triển khai production. Bước cần cho nghiệm thu tiếp theo là annotation độc lập các giao cắt và đánh giá association/coverage theo ngưỡng thống nhất; không mở rộng sang điều khiển bay.

## 8. File đã kiểm tra và thay đổi

Đã kiểm tra working tree, `AGENTS.md`, `PROGRESS.md`, `README.md`, `docs/system_contract.md`, các skill bug-fix/runtime-verification/phase-review, world test, launcher, observer, appearance/identity candidate, replay/renderer, regression tests, log/config/cache/CSV/video của run cuối.

Trong lượt hoàn tất này:

- Tạo `evaluation/audit_tracking_identity.py`: audit đọc bằng chứng, chấm accuracy proxy/unknown/false assignment/ID dư, kiểm tra replay bằng live run và dựng loss intervals đúng.
- Tạo `evaluation/tracking_loss_intervals.py`: đóng interval đúng actor.
- Tạo `evaluation/test_audit_tracking_identity.py`: regression causal anchor, mẫu số đúng/sai/unknown, ID dư và loss đồng thời.
- Sửa `evaluation/tracking_3_people_test.py`: lỗi đóng loss event và mô tả label proxy.
- Sửa `evaluation/check_tracking_reentry.py`: kiểm tra assignment bẩn/PID cũ/TTL và xuất sang thư mục mới.
- Tạo báo cáo này và `completion_audit/` chứa số liệu/ảnh/log kiểm tra bổ sung.

Giữ nguyên tracker candidate, descriptor, replay, launcher, world, mọi video/summary/cache cũ, tài liệu milestone lịch sử và file production ngoài phạm vi. `launch_phase4a_review.sh` chạy luồng production, không chọn guarded test; đây là luồng khác cần phân biệt, không xóa/sửa.

## 9. Kiểm chứng và lệnh

| Kiểm tra | Kết quả |
|---|---|
| Lượt Gazebo → YOLO26n → DeepSORT → PID đã lưu | PASS, hoàn tất và hạ cánh |
| Hash nguồn/cache, CSV, live = replay, video frame cuối | PASS |
| Regression identity và audit/loss | PASS — 14/14 |
| Reentry ngắn/dài, assignment lúc vắng và TTL | PASS |
| Python compile, bash syntax, diff whitespace | PASS |
| Full Gazebo chạy lại sau sửa bookkeeping | NOT RUN — association không đổi; tái hiện bookkeeping bằng frame log thật |
| Nghiệm thu DeepSORT TID | FAIL |
| Nghiệm thu toàn bộ PID | INCONCLUSIVE |

Từ thư mục `project`, chỉ một terminal; launcher tự khởi động các thành phần và dừng sạch. Không cần chạy lại simulation để xem bằng chứng hiện có.

```bash
# Kiểm tra logic và đọc bằng chứng đã hoàn tất:
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ../software/venvs/yolo26/bin/python -m unittest discover -s evaluation -p 'test_*tracking_identity*.py' -v
python3 -m py_compile evaluation/tracking_3_people_test.py evaluation/audit_tracking_identity.py evaluation/tracking_loss_intervals.py evaluation/check_tracking_reentry.py
bash -n simulation/run_tracking_3_people_test.sh
git diff --check

# Nếu tái audit: dùng đường dẫn output mới, không ghi đè artifact cũ.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ../software/venvs/yolo26/bin/python evaluation/audit_tracking_identity.py evaluation/results/runs/tracking_3_people_20261009T084549Z --output evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/recheck_metrics.json
python3 evaluation/check_tracking_reentry.py evaluation/results/runs/tracking_3_people_20261009T084549Z --output-dir evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/recheck_reentry

# Tái thí nghiệm khi cần dữ liệu mới:
TRACKING_PROFILE=guarded TRACKING_WORLD=tracking_3_people_reentry_test TRACKING_STOP_SIM_TIME=234 ./simulation/run_tracking_3_people_test.sh
```

Lệnh replay/renderer cho capture mới có trong [báo cáo triển khai trước](tracking_identity_2026-10-09.md). Video chính: [`identity_comparison/comparison.mp4`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/identity_comparison/comparison.mp4); log gốc: [`observer.log`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/observer.log), [`identities.csv`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/identities.csv), [`frames.csv`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/frames.csv). Bằng chứng tests: [`completion_audit/unit_tests.log`](../../evaluation/results/runs/tracking_3_people_20261009T084549Z/completion_audit/unit_tests.log).
