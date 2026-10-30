# TurtleBot 4 — Bài thực hành 7

Phân vùng map maze thành A/B/C, nhận câu tiếng Việt từ micro laptop và gửi `NavigateToPose` cho Nav2. Sinh viên: Đỗ Việt Anh — 23020719 — K68E-RE.

## Môi trường

Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Standard, Ignition Fortress, Nav2, Python 3.10. Dùng desktop X11 cho Gazebo/RViz và PulseAudio cho microphone. Node giọng nói không publish `/cmd_vel`.

Cài ROS Humble và simulator theo [TurtleBot 4 Simulation](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html), sau đó:

```bash
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-opencv python3-numpy python3-scipy \
  python3-yaml python3-matplotlib python3-tk python3-venv pulseaudio-utils xdotool ffmpeg
```

Các lượt kiểm tra dùng Create 3 simulation overrides từ [Bài 04](https://github.com/vanhhhhh512/turtlebot4_bai04), commit `9f2e4143fcf0caddd72a630accf6f99a6bf37cc5`. Giữ dependency này ngoài repo Bài 07:

```bash
mkdir -p ~/bai07_underlay/src
cd ~/bai07_underlay/src
git clone https://github.com/vanhhhhh512/turtlebot4_bai04.git bai04_turtlebot4
git -C bai04_turtlebot4 checkout 9f2e4143fcf0caddd72a630accf6f99a6bf37cc5
cd ~/bai07_underlay
source /opt/ros/humble/setup.bash
colcon build --base-paths src/bai04_turtlebot4/upstream \
  --packages-select irobot_create_nodes irobot_create_control \
  --allow-overriding irobot_create_nodes irobot_create_control \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

ASR có thể dùng Python riêng, không thay dependency của các bài khác:

```bash
python3 -m venv --system-site-packages ~/.venvs/bai07-asr
~/.venvs/bai07-asr/bin/pip install -r ~/bai07_ws/src/bai07_turtlebot4/requirements-asr.txt
~/.venvs/bai07-asr/bin/python -c "import whisper; whisper.load_model('small', device='cpu')"
```

Chạy lệnh cài ASR sau khi clone bên dưới. Whisper tải model `small` đa ngôn ngữ vào `~/.cache/whisper`; không dùng model `.en`. SHA256 model đã tải: `9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794`. Cấu hình hiện dùng CUDA với **FP32**; có thể đổi `device: cpu` trong `config/speech.yaml` nếu máy không có GPU. Không dùng FP16 trên GTX 1650 trong phiên kiểm tra vì đã thấy kết quả NaN.

## Clone, build và mở demo

```bash
mkdir -p ~/bai07_ws/src
cd ~/bai07_ws/src
git clone https://github.com/vanhhhhh512/turtlebot4_bai07.git bai07_turtlebot4
cd ~/bai07_ws
source /opt/ros/humble/setup.bash
source ~/bai07_underlay/install/setup.bash
colcon build --symlink-install --packages-select bai07_turtlebot4
source install/setup.bash
```

Chọn đúng source micro trong `config/speech.yaml` bằng `pactl list short sources`; không chọn đuôi `.monitor`. Bản cấu hình laptop dùng `alsa_input.pci-0000_05_00.6.analog-stereo`. Chỉnh gain bằng Settings → Sound; giảm nếu panel báo clipping. Script không tự đổi âm lượng hệ thống.

Mở mọi cửa sổ bằng một lệnh:

```bash
BAI07_UNDERLAY=~/bai07_underlay \
  ~/bai07_ws/src/bai07_turtlebot4/scripts/run_demo.sh \
  asr_python:=$HOME/.venvs/bai07-asr/bin/python
```

Nếu dùng Python ASR đã cài sẵn ở user/system, bỏ argument `asr_python`. Script tạo run mới, lưu `session.env`, mở Gazebo bên phải trên, transcript bên phải dưới và RViz bên trái. Cần mở khóa màn hình. Chờ `READY` của robot và ASR. Node chuẩn bị kích hoạt controller, undock rồi khởi tạo AMCL một lần từ spawn (0,0,0) + odom. Navigation tiếp tục dùng AMCL/scan/map/TF; ground truth không dùng làm đầu vào.

Mặc định ROS domain **97**, partition Ignition mới mỗi phiên. Script dùng Cyclone DDS với discovery localhost theo `config/cyclonedds.xml`; `session.env` lưu cả cấu hình DDS cho terminal kiểm tra. Kiểm tra phiên khác trước khi chạy, đổi bằng `BAI07_ROS_DOMAIN_ID`/`BAI07_IGN_PARTITION` nếu trùng. Ctrl+C ở terminal launch đóng đúng phiên đó. Không dùng lệnh kill rộng.

## Nói và quan sát

Bấm **Nghe một câu**, đợi dòng **Nói câu lệnh…**, nói rồi ngừng khoảng một giây. Micro có bước ổn định/nghe nền trước mỗi câu. Panel hiện transcript, task, trạng thái và sai số đích.

| Câu nói | Task |
| --- | --- |
| Đi đến khu vực A | Tây Bắc |
| Đi đến khu vực B | Tây Nam |
| Đi đến khu vực C | Phía Đông |
| Di chuyển từ khu vực A đến khu vực B | Đến A thành công rồi mới gửi B |
| Đi đến khu vực Z | Báo không tồn tại; không gửi goal |
| Di chuyển từ khu vực A đến khu vực Z | Từ chối toàn câu, không đi đến A |

Có alias “phía tây bắc”, “phía tây nam”, “phía đông”, và cách đọc chữ “bê”, “xê”. Alias ASR “ái” được đăng ký rõ cho A sau lượt micro; parser vẫn tra alias chính xác, không fuzzy-match tên lạ. Không gán tên phòng cho các vùng vị trí này. Khi đang chạy, câu đi mới báo BUSY. Cùng utterance ID chỉ được xử lý một lần. Nút **Hủy task** gửi action cancellation, chờ result trước khi cho task mới. Im lặng sau câu đã nhận không hủy task. Mất microphone/ASR thật, mất sensor/TF, hazard nguy hiểm hoặc timeout làm hủy chuỗi; không tự gửi tiếp B hoặc tự thử lại.

Vận tốc và tránh vật cản do Nav2 xử lý. Mỗi goal cần status `SUCCEEDED`, sai số vị trí ≤0,30 m, góc ≤0,30 rad và pose cuối thuộc đúng mask vùng. Nav2 goal checker dùng 0,25 m/rad. BACKUP_LIMIT=0 sau undock không được tính thành bump; BUMP/CLIFF/STALL/WHEEL_DROP được ghi và hủy task.

## Phân vùng và dữ liệu

`maps/map.yaml`/`map.pgm` giữ nguyên byte Bài 04. Navigation dùng `config/map_navigation.yaml` với `free_thresh=0.196`, nên gray 205 là **unknown**. Grid chạy thật được so khớp với mask phân vùng. Map: 401×401, 0,05 m/pixel, origin [-10,-10,0].

Phương pháp cuối: polygon thủ công trong `config/region_seeds.yaml`, cắt với thành phần free lớn nhất. A: x<0,y≥0; B: x<0,y<0; C: x≥0. 257 ô free nhỏ không gán nhãn; tường/unknown không có nhãn. Các vùng không phải connected components của phòng. Thử seed distance-transform/watershed tạo 41 seed nên không chọn kết quả đó.

```bash
python3 ~/bai07_ws/src/bai07_turtlebot4/bai07_turtlebot4/map_segmentation.py
python3 -m unittest discover -s ~/bai07_ws/src/bai07_turtlebot4/tests -v
```

| Vùng | Goal (x,y,yaw) | Clearance map |
| --- | --- | --- |
| A | (-3.525, 2.975, 0) | 1.692 m |
| B | (-3.525, -3.375, 0) | 0.550 m |
| C | (4.625, -0.525, 0) | 0.550 m |

Đổi tâm pixel: qx=(u+0,5)r; qy=(H-v-0,5)r; rồi xoay theo yaw origin và cộng origin. Lưu mask riêng `maps/region_labels.npy`; ảnh `maps/regions.png` chỉ minh họa, không đưa vào map server. Robot radius cấu hình 0,175 m.

## Ghi log và quay demo thật

Mỗi run lưu WAV, `speech.jsonl`, `navigation.jsonl`, `trajectory.csv` và log launch. `sensors/` lưu laser, snapshot costmap và ground truth chỉ để đối chiếu nghiệm thu; recorder này không gửi pose/goal/vận tốc. Chuỗi ID nối audio → transcript → command → UUID action → result. ASR `small`, language `vi`, task `transcribe`; thu PCM mono 16 kHz bằng `parec`, bỏ transient mở mic, highpass 100 Hz, ngưỡng năng lượng theo nền, chuẩn hóa mức âm ASR và gợi ý từ vựng khu vực A/B/C, kiểm tra clipping/no-speech/logprob/compression trước khi phát câu cuối. ASR chạy worker riêng và chỉ nhận một câu mỗi lần bấm.

Quay video ở terminal thứ hai (script từ chối màn hình khóa, không ghi đè video cũ):

```bash
source ~/bai07_ws/install/setup.bash
source /duong/dan/run/session.env
python3 ~/bai07_ws/src/bai07_turtlebot4/scripts/record_demo.py "$BAI07_RUN/demo_microphone.mp4"
```

Video thu trực tiếp màn hình và micro PulseAudio; Ctrl+C để chốt MP4. Nói A/B/C, sau C nói A→B, cuối cùng nói Z. Chờ robot hoàn thành từng task; không dùng teleop hay gõ text thay câu nói. Có thể dùng OBS nếu cần layout/nguồn âm khác. Chỉ công bố video sau khi xem và kiểm tra tiếng/hình.

## Kiểm tra bằng text và vật cản

Các script dưới đây là **đầu vào thử**, không chứng minh ASR/micro:

```bash
source ~/bai07_ws/install/setup.bash
source /duong/dan/run/session.env
python3 ~/bai07_ws/src/bai07_turtlebot4/scripts/send_text_test.py 'Đi đến khu vực A'
python3 ~/bai07_ws/src/bai07_turtlebot4/scripts/run_text_checks.py --output "$BAI07_RUN/text_checks.json"
```

`maze_obstacle.py create/remove --output FILE` thêm/xóa box thật trong **partition đang source**; các tùy chọn `--x --y --size --name` xác định vật cản. Dùng để kiểm tra laser/costmap/đường vòng hoặc goal bị chặn. Không tính thử có va chạm/hủy là lượt đến đích thành công.

Topic: `/recognized_text` (String JSON: `utterance_id`, `text`, `created_at` Unix giây, `source` microphone/text_test/audio_test; micro thêm audio_path/record_start/record_end/asr_s), `/speech_status`, `/voice_navigation_status`, `/semantic_regions`, `/voice_actual_path`. Lệnh thiếu schema hoặc quá 30 s bị từ chối. `/speech_control`: String `listen`; `/voice_cancel`: String bất kỳ.

## File chính và giới hạn nghiệm thu

`map_segmentation.py` tạo vùng/goal; `speech_recognition_node.py` thu/ASR; `command_parser.py` lấy nguồn/đích; `voice_navigation_node.py` quản lý action; `semantic_map_node.py` hiển thị vùng; `prepare_robot.py` undock/initial pose; `demo_panel.py` hiển thị transcript live.

Đã chạy navigation thật bằng text cho A/B/C và C→A→B. Phần micro tiếng Việt và video cuối chỉ được ghi đạt khi có người nói thật và lưu đủ evidence. Repo không chứa báo cáo, log, WAV, video hoặc model. Chưa có URL video Bài 07 được xác minh.
