# voicevox_ros
VOICEVOXをROS1で手軽に実行できるようROSパッケージにしました。  
Ubuntu20.04 + ROS Melodicで動作確認をしています。

## 環境構築
1. ROSのインストール
2. VOICEVOXのインストール
VOICEVOXの公式サイトからVOICEVOXをインストールしてください。  
https://voicevox.hiroshiba.jp/
3. voicevox_rosのインストール
rosワークスペースのsrc領域ににvoicevox_rosをコピーしてcatkin_makeを行ってください。
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/tosiyuki/voicevox_ros.git
cd ..
catkin_make
```
4. ライブラリのインストール
```
pip install -r requirements.txt
```

## サンプルコードの実行
- ターミナル1
    ```
    ~/.voicevox/VOICEVOX.AppImage
    ```
- ターミナル2
    ```
    cd ~/catkin_ws/src
    source devel/setup.bash
    roslaunch voicevox_ros voicevox_ros.launch
    ```
- ターミナル3
    ```
    cd ~/catkin_ws/src
    source devel/setup.bash
    rosrun voicevox_ros sample.py
    ```
実行後、ターミナル3にテキストを入力すると音声が出力されます。
