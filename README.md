# RoboMaster_tutorial
RoboMaster AI Challenge に向けての個人的な練習とかあんまり関係ないもの置くとこ.

## Contents

### [dual_lidar_robot](dual_lidar_robot)
前後にLiDAR(検出角度180 deg)を配置した差動2輪ロボットを動かす.   
rvizにLiDARのデータが表示される.赤が生データ, 白が2コのLiDARのデータを合成しロボット座標にtfしたデータである.   

#### 使い方
通常のロボットは, 
``` console
roslaunch dual_lidar_robot main.launch
```
でプログラムが起動する. 起動したコンソール上で矢印キーを押すとロボットが動く.

LiDARの高さを変えられるロボットは,
``` console
roslaunch dual_lidar_robot move_lrf.launch
```
でプログラムが起動する. ウィンドウを切り替えてjoint_states_publisherからスライドバーを調整するとLiDARの高さが変化する. その他は通常のロボットと同じである.
