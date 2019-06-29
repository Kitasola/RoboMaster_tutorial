# tf2
tf2に関するリファレンスのようなもの.

## Broadcaster
tfとの相異点は無し. 型名を変更するだけでよい.

## Static Broadcaster
comming soon...

## Listener
tfで2フレーム間の位置姿勢を取得する場合
``` cpp
try {
  ros::Time now = ros::Time::now();
  listener.waitForTransform("/turtle2", "/turtle1", now, ros::Duration(3.0));
  listener.lookupTransform("/turtle2", "/turtle1", now, transform);
}
```
と書けるが, tf2の場合
``` cpp
try {
  transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time::now(), ros::Duration(3.0));
}
```
のように2つを統合した関数が存在する. ros::Duration()でタイムアウトまでの時間を指定できる.

***

tfで使われていたtransformPointCloudなどの[座標変換の関数](http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1TransformListener.html)は, tf2では[doTransform]()のみとなっている(実態はtemplateを使って複数の型に対応している).

### PointCloud2
また tf2でsensor_msgs::PointCloudは座標変換できない. 代わりに後継のsensor_msgs::PointCloud2なら座標変換できる.   
ただ, PointCloud2の構造は複雑なので, 別の型で作成したデータをPointCloud2に変換した方が楽である. [このプログラム](../dual_lidar_robot/src/synthesis_lrf.cpp)では, sensor_msgs::PointCloudに構造が近いpcl::PointCloud<pcl::PointXYZ>を用いた.
