# tc2017_RAB_src
icartベースのベビーカーとイーサーネット版のTop-URGを2つ使用しています。

## 起動手順
1. ypspurの起動
```bash
$ roslaunch ypspur_ros ypspur_rab.launch
```

2. urg_nodeとlrf_mergerの起動
```bash
$ roslaunch rab_utility_pkg urg_merger.launch
```

3. joyの起動
```bash
$ roslaunch rab_utility_pkg joy_teleop.launch
```
4.mappingのためのログ取り
以上１〜３を起動した状態で
```bash
$ cd {log}
$ rosbag record -a
```
{}には任意の場所を入れること
オススメはlogのなかにもなんのログかわかるように場所名を入れる ex)log/tc2017
終わったらctrl-cで終了させる。

5.mapの作成
```bash
$ roscd nda_robot_2dnav
$ cd map
$ rosparam set use_simtime true
$ rosrun gmapping slam_gmapping_play --scan_topic=/base_scan --bag_filename=hogenboku(ログを作成したいbagファイル)
```

6.mapの保存
```bash
$ rosrun map_server map_saver -f {filename}
```
作成したmapのファイル名をnda_robot_2dnav内のmove_base.launchに入れる。

7.navigation起動
1と2を起動した状態で、
```bash
$ roslaunch nda_robot_2dnav move_base.launch
```