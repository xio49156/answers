## challenge_7_2
プログラムの編集部分を全て記述しておきます．  
１．challenge_7_2.pyは「chapter7/pseudo_node/pseudo_node/challenge_7_2.py」に配置  

２．challenge_7_2.pyはchapter3のspeech_service.pyをコピペ  

３．challenge_7_2.py：20行目  
'/speech_service/wake_up'  
を  
'voice/command'  
に編集（ /speech_service/wake_up -> voice/command ）  

４．chapter7/pseudo_node/setup.py：26行目  
'voice_node = pseudo_node.voice_node:main'  
から  
'voice_node = pseudo_node.challenge_7_2:main'  
へ編集（ voice_node -> challenge_7_2 ）  

現状は認識した音声が何であろうと，次のステートへ進むというプログラムになっています．  
（何も音声を認識しなければステートは進みません）  
そのため，認識した音声から特定の物体名や場所名を見つけるためには，  
chapter7/bringme_sm/bringme_sm/bringme_sm.py  
のVoiceクラスに存在する94, 95行目の  
find_object_name(response.answer)  
find_location_name(response.answer)  
を新たに関数として追加する必要があります．  
Challenge3.1を参照


後日綺麗に編集
