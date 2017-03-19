# drone_KF
ARドローンより得られるIMUデータを用いて、姿勢角をカルマンフィルタにより推定する。　　

## 環境構築

ドローンのシミュレータ関係一式
```bash
git clone git@github.com:lancer-evolution/ardrone.git
```

## 実行

シミュレータとコントローラを起動する。
```bash
roslaunch cvg_sim_gazebo ardrone_testworld.launch
rosrun tum_ardrone drone_gui
```

カルマンフィルタの起動
```bash
rosrun drone_KF imu
```

比較するためのグラフ
```bash
rqt_plot
```

![](plot2.png)

## トピックリスト

* /imu/estimate
* /imu/measure

## 現在

現在はroll角のみ推定している。

## 状態方程式

```math
x_{k+1}=Ax_k+Bu_k
```