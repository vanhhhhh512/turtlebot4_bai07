# TurtleBot 4 — Bài thực hành 7

Phân vùng map maze thành 1/2/3, nhận câu tiếng Việt từ micro laptop và gửi `NavigateToPose` cho Nav2.

## Môi trường

Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Standard, Ignition Fortress, Nav2, Python 3.10. Dùng desktop X11 cho Gazebo/RViz và PulseAudio cho microphone. Node giọng nói không publish `/cmd_vel`.

Cài ROS Humble và simulator theo [TurtleBot 4 Simulation](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html), sau đó:

```bash
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-opencv python3-numpy python3-scipy \
  python3-yaml python3-matplotlib python3-tk python3-venv pulseaudio-utils xdotool ffmpeg
```
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

```

## Nói và quan sát

Bấm **Nghe một câu**, đợi dòng **Nói câu lệnh…**, nói rồi ngừng khoảng một giây. Micro có bước ổn định/nghe nền trước mỗi câu. Panel hiện transcript, task, trạng thái và sai số đích.

| Câu nói | Task |
| --- | --- |
| Đi đến khu 1 | Tây Bắc |
| Đi đến khu 2 | Tây Nam |
| Đi đến khu 3 | Phía Đông |
| Di chuyển từ khu 1 đến khu 2 | Đến 1 thành công rồi mới gửi 2 |
| Đi đến khu 4 | Báo không tồn tại; không gửi goal |
| Di chuyển từ khu 1 đến khu 4 | Từ chối toàn câu, không đi đến 1 |

Parser nhận cả chữ số và chữ tiếng Việt: “khu 1/khu một”, “khu 2/khu hai”, “khu 3/khu ba”; có thể nói “khu vực số hai”. ID nội bộ cũng là chuỗi "1", "2", "3", không dùng chữ cái. Tên cũ không còn là alias điều khiển. Parser tra alias chính xác, không fuzzy-match tên lạ; số 4/bốn hoặc tên ngoài cấu hình bị từ chối. Không gán tên phòng cho các vùng vị trí này. Khi đang chạy, câu đi mới báo BUSY. Cùng utterance ID chỉ xử lý một lần. Nút **Hủy task** gửi cancellation và chờ result trước task mới. Mất microphone/ASR, sensor/TF, hazard nguy hiểm hoặc timeout hủy chuỗi; không tự gửi goal kế tiếp hoặc tự thử lại.

Vận tốc và tránh vật cản do Nav2 xử lý. Mỗi goal cần status `SUCCEEDED`, sai số vị trí ≤0,30 m, góc ≤0,30 rad và pose cuối thuộc đúng mask vùng. Nav2 goal checker dùng 0,25 m/rad. BACKUP_LIMIT=0 sau undock không được tính thành bump; BUMP/CLIFF/STALL/WHEEL_DROP được ghi và hủy task.

```

| Vùng | Goal (x,y,yaw) | Clearance map |
| --- | --- | --- |
| 1 | (-3.525, 2.975, 0) | 1.692 m |
| 2 | (-3.525, -3.375, 0) | 0.550 m |
| 3 | (4.625, -0.525, 0) | 0.550 m |
