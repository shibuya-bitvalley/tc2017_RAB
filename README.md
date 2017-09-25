<<<<<<< HEAD
# tc2017_RAB_src
icartベースのベビーカーとイーサーネット版のTop-URGを2つ使用しています。

## 起動手順
1. ypspurの起動
```bash
$ sh nishidalab_ypspur_start.sh
$ roslaunch nishidalab_ypspur_driver nishidalab_ypspur_driver.launch
```

2. urg_nodeとlrf_mergerの起動
```bash
$ roslaunch icart_pkg urg_merger.launch
```

3. joyの起動
```bash
$ roslaunch icart_pkg joy_teleop.launch
```
=======
# human_recognition
Human recognition project for Tsukuba Challenge.
- record videos by multi cameras
- extract images from the video
- create overlayed image dataset
- train neural networks with chainer
- load trained chainer model and search humans
>>>>>>> e04e5e31cd53e8d277ec7b2aadf50cf7665aa2ef
