# tc2017_RAB_src
icartベースのベビーカーとイーサーネット版のTop-URGを2つ使用しています。

## 起動手順
1. ypspur_rosの起動
```bash
$ roslaunch ypspur_ros ypspur_rab.launch
```

2. urg_nodeとlrf_merger(未完成)の起動
```bash
$ roslaunch icart_pkg urg_merger.launch
```

3. joyの起動
```bash
$ roslaunch icart_pkg joy_teleop.launch
```
