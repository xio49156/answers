## 7章の解答例
### challenge_7_1.py
- このフォルダの```challenge_7_1.py```ファイルを，```~/airobot_ws/src/chapter7/sample_sm/sample_sm```にコピーする．

- ```~/airobot_ws/src/chapter7/sample_sm/setup.py``` の entry_pointsの設定のリストに以下の行を追加する．
```
            'challenge_7_1 = sample_sm.challenge_7_1:main'
```

- 端末で以下を実行
```
cd ~/airobot_ws
source install/setup.bash
```

- 端末で以下を実行
```
ros2 run sample_sm challenge_7_1
```

### challenge_7_2.py
- このフォルダの```challenge_7_2.py```ファイルを，```~/airobot_ws/src/chapter7/pseudo_node/pseudo_node```にコピーする．
- ```~/airobot_ws/src/chapter7/pseudo_node/setup.py```の```entry_points```の設定のリストに以下の変更を行います．
```
            'voice_node = pseudo_node.voice_node:main'
```
から
```
            'voice_node = pseudo_node.challenge_7_2:main'
``` 

### 備考
現状は認識した音声が何であろうと，次のステートへ進むというプログラムになっています（何も音声を認識しなければステートは進みません）．
そのため，認識した音声から特定の物体名や場所名を見つけるためには，
```chapter7/bringme_sm/bringme_sm/bringme_sm.py```
のVoiceクラスに存在する94, 95行目の  
```find_object_name(response.answer)```
```find_location_name(response.answer)```
を新たに関数として追加する必要があります．  
```Challenge3.1```を参照
