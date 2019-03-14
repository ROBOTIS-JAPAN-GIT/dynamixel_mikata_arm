# DYNAMIXEL MikataArm
**PLEASE ALSO CHECK THE NEWEST REPOSITORY AT https://github.com/ROBOTIS-JAPAN-GIT/open_manipulator**

The ROS-enabled Open Source Robotics Arm for education, research, competition (e.g. RoboCup@Home).

This repository provides an install guide and sample programs.

ROS対応ロボット専用アクチュエーターDYNAMIXEL(ダイナミクセル) Xシリーズで構成するオープンソースのロボットアームです。
教育・研究・競技用。 ロボカップ＠ホームなど。


DYNAMIXEL Xシリーズ e-マニュアル :http://support.robotis.com/en/product/actuator/dynamixel_x_main.htm 

Contact: jp.support@robotis.com

## 動作環境

このサンプルプログラムではDYNAMIXEL XM430-W350、あるいは XH430-V350が使用されていることを想定しています。それ以外の製品では弊社 e-Manual の各機種のコントロールテーブル (Control Table) を確認の上、必要な箇所を変更して下さい。尚、各モーターのIDはベースリンク(基台取り付け側)から順に1から5を前提としています。

Ubuntu 16.04 (x86_64), ROS kinetic, Dynamixel SDK 3.4.7 で動作検証しています。これ以外の環境では一部のコードの変更が必要になる可能性があります。

ご質問・ご意見などがありましたら、遠慮なくご連絡下さい。


## インストレーションガイド

MikataArmを初めて使用する際は、以下の手順に沿ってソフトウェアのインストールと初期設定を行ってください。

1. ROS kinetic をインストールします。
   http://wiki.ros.org/kinetic/Installation/Ubuntu の指示に従って `ros-kinetic-desktop` をインストールします

2. 依存パッケージをインストールします。
```
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-dynamixel-sdk ros-kinetic-jsk-rviz-plugins python-catkin-tools libeigen3-dev
```

3. catkinワークスペースを作成し、本リポジトリをcloneしてbuildします。
```
mkdir -p ~/mikata_arm_ws/src
cd ~/mikata_arm_ws/src
catkin_init_workspace
git clone https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm.git
cd ..
catkin build
```

4. USB2DXLの読み書き許可とlatency_timerを設定するため、udev rules を追加します。
```
cd ~/mikata_arm_ws
sudo cp src/dynamixel_mikata_arm/99-ftdi_sio.rules /etc/udev/rules.d/
```

5. 実機とPCを接続します。すでにUSB2DXLを接続していた場合は、挿し直してください。

6. インストレーションスクリプトを実行します。
```
cd ~/mikata_arm_ws
./src/dynamixel_mikata_arm/install.sh
```

スクリプトがエラーなく終了すると、初期設定は完了です。


## サンプルプログラムの実行

サンプルプログラムを実行するために、パッケージのsetup.bashをsourceします。
```
cd ~/mikata_arm_ws
source ./devel/setup.bash
```

次に、roslaunch や rosrun で各プログラムを実行します。bringup ノードを立ち上げると、ROSのトピックやサービスで MikataArm を動かすことができます。cli_control では、コマンドラインインターフェースにより実機を動かせたり、ティーチングやIKのデモを実行することができます。

### 実行例:

* ROS GUI版のサンプルプログラムを実行
  - mikata_armのbringupノードを起動。
    ```
    roslaunch mikata_arm_bringup bringup.launch gui:=true
    ```
  - bringup.launch の起動を待った後、rvizを起動。
    ```
    roslaunch mikata_arm_bringup rviz.launch
    ```
    
* CLI版のサンプルプログラムを実行

    ```
    $ rosrun mikata_arm_toolbox cli_control
    ```
    
* 注意) ROS GUI版、CLI版の２つを同時に実行することはできません。
  


## サンプルプログラム紹介

### mikata_arm_bringup:

* **bringup.launch** : ROSのサンプルプログラムです。MikataArm を操作するためのトピックやサービスを立ち上げます。'gui' オプションで簡単なGUI操作が可能となります。

* **rviz.launch** : MikataArm をrvizで表示します。実行にはbringup.launchが必要となります。

* **motion_player_rviz.launch** : MikataArm と共に、 MotionPlayer に関する情報をrvizで表示します。


### mikata_arm_toolbox:

* **cli_control** : コマンドラインインターフェースにより MikataArm を操作できます。各アクチュエータへの指令・情報取得だけでなく、ティーチングや逆運動学の例もあります。

* **dxl_factory_reset** : 全DYNAMIXELの初期化。

* **dxl_setup** : MikataArm の構造に合わせてDYNAMIXELの設定を変更します。cli_controlの実行には必要になります。
