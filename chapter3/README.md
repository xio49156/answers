# ダミー

- このフォルダの```.py```ファイルを，```~/airobot_ws/src/chapter3/speech_service/speech_service```にコピーする．
- ```~/airobot_ws/src/chapter3/speech_service/setup.py```の```entry_points```の設定のリストに以下の行を追加する．
```
            'challenge_3_1 = speech_service.challenge_3_1:main',
```

- 端末で以下を実行
```
cd ~/airobot_ws
source install/setup.bash
```

- 以下のコマンドで```challenge_3_1```を立ち上げます．
```
ros2 run speech_service challenge_3_1
```
- そのあとに，別の端末を立ち上げて，challenge_3_1を動かすためのコマンドを実行します．
```
ros2 service call /speech_service/wake_up airobot_interfaces/srv/StringCommand
```
