# nda_robot_pkgs
NDAのROSパッケージです．

## インストール方法

以下，`<catkin_ws>`と書いてある部分は任意のワークスペースのパスに読み替えてください．

### 1. 本リポジトリをワークスペースに`clone`する
##### HTTPを利用する場合
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/Nishida-Lab/nda_robot_pkgs.git
```
##### SSHを利用する場合
```bash
$ cd <catkin_ws>/src
$ git clone git@github.com:Nishida-Lab/nda_robot_pkgs.git
```

### 2. `wstool`コマンドによる依存パッケージ(apt-getできないもの)のダウンロード
```bash
$ cd <catkin_ws>
$ wstool init src
$ wstool merge -t src src/nda_robot_pkgs/dependencies.rosinstall
$ wstool update -t src
```
### 3. `rosdep`コマンドによる依存パッケージ(apt-getできるもの)のダウンロード
```bash
$ cd <catkin_ws>
$ rosdep install -i -r -y --from-paths src --ignore-src
```

### 4. `catkin_make`の実行とパス通し
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```
