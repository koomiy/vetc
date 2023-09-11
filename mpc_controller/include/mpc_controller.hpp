#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/sf_to_mpc.h>
#include <custom_msgs/mpc_bw_nlp.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

#define MACHINE_ACTUATION 1

using namespace Eigen;

class mpc
{
public: 
    mpc();
    void init();
    void spin();

private: 
    void observer(const double& theta, const double& V);   // 
    void sensorCallback(const custom_msgs::sf_to_mpc& sub_sf_mpc);    // ２つのセンサーから角度と目標角度の両方を検出
    void control();
    void actuate();

    // ハンドラと、パブリッシャ・サブスクライバの定義
    ros::NodeHandle    nh;
    ros::Subscriber    sub_angles;
    ros::ServiceClient client;
    ros::Publisher     pub_angle;
    ros::Publisher     pub_input;

    // 環境変数
    int count;
    int controlCycle;
    double dt;
    double time;
    bool machine_actuation;

    // パラメータ
    double J;
    double D;
    double Kt;
    double Ksp;
    double tau0;
    double Re;
    double L;
    double Ke;
    double Ng;

    // 独立変数
    Vector2d x;     // 状態の目標偏差
    double u;       // 入力電圧の目標偏差

    double target_theta;    // 目標角度<rad>
    double theta;           // 現在角度<rad>
    double dtheta;          // 現在角速度<rad/s>
    double ddtheta;         // 現在角加速度<rad/s^2>
    double V;               // 現在電圧<V>
    double I;               // 現在電流<A>
    

};

/*class timer // 時間に関する更新はここで一括で行いたい
{
public: 
    timer();

    int count;
    int controlCycle;
    double dt;
    double time;
};*/

class param // パラメータの設定はここで一括でやりたい
{
public: 
    double Jm = 0.1;
    double Jv = 0.9;
    double J = Jm + Jv;
    double Dm = 0.9;
    double Dv = 0.1;
    double D = Dm + Dv;
    double Kt = 1.0;
    double Ksp = 1.0;
    double tau0 = 1.0;
    double Re = 1.0;
    double L = 1.0;
    double Ke = 1.0;
    double Ng = 1.0;
};