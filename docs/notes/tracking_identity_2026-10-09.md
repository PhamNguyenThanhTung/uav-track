# Kiểm chứng duy trì danh tính — 2026-10-09

## Kết luận và chính sách

Nên tách **TID** (track ngắn hạn của DeepSORT) khỏi **PID** (danh tính tạm trong phiên). Lưu lịch sử ID để đánh giá, nhưng không nối lại một người vô thời hạn. Bản thử nghiệm giữ gallery trong **15 giây thời gian mô phỏng** kể từ quan sát được chấp nhận: quay lại trong hạn chỉ khôi phục PID khi ngoại hình khớp rõ ràng, không có ứng viên cạnh tranh và chuyển động hợp lý. Khi mơ hồ, bỏ trống PID; khi hết hạn, cấp PID mới. Không xuất quan sát dự đoán thành người đang hiện diện khi người đã ra khỏi ảnh. PID không được tái sử dụng trong phiên; chưa lưu gallery xuyên lần khởi động.

15 giây là tham số thử nghiệm, chưa phải ngưỡng tối ưu cho bay thực tế. Tăng riêng `max_age` không giải quyết đổi danh tính: trong phép so sánh dưới đây nó làm chỉ số đổi ID tăng.

## Thay đổi chỉ trong luồng thử nghiệm

- Kết hợp embedding MobileNet với histogram HS vùng thân, không dùng nhãn actor hoặc ngưỡng màu riêng cho từng người.
- Kiểm tra ngoại hình cả ở bước ghép IoU, xác nhận track qua 3 quan sát.
- Không cập nhật gallery bằng crop che khuất mạnh, độ tin cậy thấp hoặc sát mép ảnh. Điều kiện mép ảnh khắc phục việc cấp PID mới từ phần người vừa ló vào ảnh.
- Ghép PID một-một bằng Hungarian, có ngưỡng khoảng cách, chênh lệch ứng viên và kiểm tra chuyển động ngắn hạn; gallery cập nhật chậm từ crop sạch.
- Ghi đồng thời PID/TID, cache detection/embedding và video camera để replay các cấu hình trên cùng dữ liệu.

Các file mới: `evaluation/tracking_appearance.py`, `tracking_identity_candidate.py`, `replay_tracking_identity.py`, `render_identity_comparison.py`, `check_tracking_reentry.py`, `test_tracking_identity_candidate.py` và `simulation/worlds/tracking_3_people_reentry_test.sdf`. Observer `evaluation/tracking_3_people_test.py` và launcher `simulation/run_tracking_3_people_test.sh` được mở rộng. Luồng production, chọn mục tiêu và điều khiển drone không được tích hợp thay đổi này. Launcher test mặc định vẫn dùng baseline; guarded là tùy chọn rõ ràng.

## Bằng chứng chạy trực tiếp

Run: `evaluation/results/runs/tracking_3_people_20261009T084549Z`, 3.166 frame camera, thời gian mô phỏng 36,964–234,204 giây. `cleanup_status.txt`: `landed=true`, `observer_exit=0`.

| Tình huống | Vắng khỏi ảnh | PID trước → sau | TID trước → sau | Kết quả |
|---|---:|---|---|---|
| Ra ngoài ngắn | 8,316 s | 3 → 3 | 90 → 106 | PASS |
| Ra ngoài dài | 27,128 s | 3 → 4 | 106 → 129 | PASS |

Cả hai khoảng vắng đều có 0 quan sát xuất ra cho người C. Kết quả máy đọc trong `reentry_validation.json`, kèm sáu ảnh trước/vắng/quay lại. Mốc vắng được xác định bằng pixel màu của người C trong kịch bản mô phỏng; đây không phải bộ đo ground truth tổng quát.

## Replay cùng dữ liệu

Chấm từ giây mô phỏng 50. Baseline: max_age=30, n_init=1; age-only chỉ đổi max_age=90. Guarded dùng max_age=60, n_init=3 cùng các thay đổi ngoại hình/gallery ở trên. `max_age` tính theo frame cập nhật; TTL của PID tính bằng giây mô phỏng.

| Cấu hình / namespace | Đổi ID theo heuristic | Đổi ID khi thấy rõ | Tỷ lệ detection có ID | Tỷ lệ khi thấy rõ |
|---|---:|---:|---:|---:|
| Baseline TID | 278 | 64 | 98,32% | 99,92% |
| Chỉ tăng max_age, TID | 313 | 74 | 99,37% | 99,92% |
| Guarded TID | 162 | 23 | 97,83% | 99,49% |
| Guarded PID | 0 | 1 | 91,88% | 98,47% |

Một lần đổi PID khi thấy rõ là cấp PID 4 sau khi hết TTL, đúng chính sách. Tỷ lệ có ID là **coverage, không phải độ chính xác**: hệ thống chủ động không gán khi thiếu tin cậy. Raw TID vẫn phân mảnh.

Chỉ số heuristic chọn ID đại diện theo nhãn màu và xét thay đổi trong khoảng 1 giây; chỉ số clear-view dùng nhãn màu chiếm ít nhất 90% và overlap không quá 10%, xét cả khoảng mất dài. Chúng không phải IDF1/HOTA hay IDSW chuẩn MOTChallenge.

Có **hai cờ đổi nhãn của cùng PID 3** tại 187,740 và 188,036 giây (C → B → C). Kiểm tra ảnh cho thấy bbox PID 3 vẫn bao người áo xanh dương nhưng lúc che khuất chứa áo xanh lá phía trước; nhãn chấm theo màu có thể bị nhiễm. Giữ nguyên các cờ này, không diễn giải con số 0 heuristic thành không bao giờ nhầm người. Ảnh `ambiguous_event_0.png` và `ambiguous_event_1.png` lưu bằng chứng.

Video `identity_comparison/comparison.mp4` hiển thị baseline TID và guarded PID/TID cạnh nhau; 12 fps nên tốc độ phát chậm hơn thời gian mô phỏng, timestamp trên ảnh là mốc đối chiếu. `comparison.json`, CSV và `replay_manifest.json` lưu kết quả, cấu hình và fingerprint nguồn/cache.

## Kiểm chứng và giới hạn

- PASS: 10 regression tests cho TTL, nối lại qua TID mới, ứng viên mơ hồ, tranh PID, đóng băng gallery, reset clock, nhảy vị trí, chặn IoU sai ngoại hình và crop mép ảnh.
- PASS: chạy trực tiếp, kiểm tra hai lần ra/vào ảnh, replay cùng observation, xuất video; drone hạ cánh và observer thoát thành công.
- Chưa kiểm chứng: người mặc giống nhau, thay đổi ánh sáng mạnh, camera chuyển động mạnh, video người thật, lưu danh tính qua phiên. MobileNet cộng màu áo chưa thay thế được mô hình person-ReID được đánh giá riêng.
- Kịch bản có áo khác màu, và proxy nhãn chấm cũng dựa màu: kết quả thuận lợi cho descriptor màu, không suy rộng thành độ chính xác thực tế.
- World vẫn là nhân vật mô phỏng; công việc này không xác nhận đã đạt độ chân thực hình người.

## Tái lập

Chạy từ thư mục `project`:

```bash
TRACKING_PROFILE=guarded TRACKING_WORLD=tracking_3_people_reentry_test TRACKING_STOP_SIM_TIME=234 ./simulation/run_tracking_3_people_test.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ../software/venvs/yolo26/bin/python evaluation/replay_tracking_identity.py evaluation/results/runs/<run>
python3 evaluation/check_tracking_reentry.py evaluation/results/runs/<run>
python3 evaluation/render_identity_comparison.py evaluation/results/runs/<run>
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ../software/venvs/yolo26/bin/python -m unittest discover -s evaluation -p test_tracking_identity_candidate.py -v
```

`launch_phase4a_review.sh` không chọn profile guarded này; dùng lệnh test bên trên để xem đúng bản thử nghiệm.

## Nguồn kỹ thuật

- [deep_sort_realtime](https://github.com/levan92/deep_sort_realtime): embedding tùy chỉnh, MobileNet mặc định và tùy chọn person-ReID.
- [DeepSORT tracker gốc](https://github.com/nwojke/deep_sort/blob/master/deep_sort/tracker.py): matching cascade, ghép IoU và vòng đời track/gallery.
- [StrongSORT](https://arxiv.org/abs/2202.13514): hướng cải thiện tracking bằng chất lượng ngoại hình và association; chưa tích hợp mô hình StrongSORT trong bản này.
