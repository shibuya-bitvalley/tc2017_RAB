# tc2017_RAB_src
icartベースのベビーカーとイーサーネット版のTop-URGを2つ使用しています。

## 起動手順
1. ypspurの起動
```bash
$ sudo sh nishidalab_ypspur_start.sh
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
