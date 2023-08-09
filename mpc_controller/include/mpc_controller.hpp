#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/sf_to_mpc.h>
#include <custom_msgs/mpc_bw_nlp.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

#define MACHINE_ACTUATION 0

using namespace Eigen;

class mpc
{
public: 
    mpc();
    void init();
    void spin();

private: 
    void sensorCallback(const custom_msgs::sf_to_mpc& sub_sf_mpc);    // ２つのセンサーから角度と目標角度の両方を検出
    void stateEq(const Vector2d& x, const double& u);
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
    double Re;
    double Ke;
    double Ng;

    // 状態方程式
    Matrix2d A;
    Vector2d B;

    // 独立変数
    Vector2d x;     // 状態エラー
    Vector2d dx;    // 状態エラーの時間微分
    double target_angle;
    double cur_angle;
    double u;   // 入力エラー

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
    double Re = 1.0;
    double Ke = 1.0;
    double Ng = 1.0;
};