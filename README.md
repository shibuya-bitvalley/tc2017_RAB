# tc2017_RAB_src
icartベースのベビーカーとイーサーネット版のTop-URGを2つ使用しています。

## インストール方法
1. workspaceの作成、git clone
```
$ mkdir -p {your_workspace_name}/src
$ cd {your_workspace_name}/src
$ catkin_init_workspace
$ git clone https://github.com/shibuya-bitvalley/tc2017_RAB.git
```

2. 九工大西田研究室作成のnda_robot_pkgsのインストール(田中さん作成)

src/nda_robot_pkgのREADMEを参照してください。

## 起動手順
1. ypspurの起動
```bash
$ roscd nishidalab_ypspur_driver/config/
$ sh nishidalab_ypspur_starter.sh 
```
別ターミナルで

```bash
$ roslaunch nishidalab_ypspur_driver nishidalab_ypspur.launch 

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
