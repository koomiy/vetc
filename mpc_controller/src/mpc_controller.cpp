#include "../include/mpc_controller.hpp"

mpc::mpc() : 
    count(0), 
    controlCycle(1), 
    dt(0.05), 
    time(0.0), 
    machine_actuation(MACHINE_ACTUATION)
{
    // サブスクライバのハンドラ立ち上げ
    sub_angles = nh.subscribe("/sf_to_mpc", 10, &mpc::sensorCallback, this);

    // サービスクライアントの設定
    client = nh.serviceClient<custom_msgs::mpc_bw_nlp>("solve");   // 引数これだけでいいのか？

    // パブリッシャのハンドラ立ち上げ
    pub_angle = nh.advertise<std_msgs::Float64>("/mpc_to_rviz", 10);
    pub_input = nh.advertise<std_msgs::Float64>("/mpc_to_pwm", 10);

}

void mpc::init(){
    // パラメータ設定
    param prm;
    J = prm.J;
    D = prm.D;
    Kt = prm.Kt;
    Ksp = prm.Ksp;
    tau0 = prm.tau0;
    Re = prm.Re;
    L = prm.L;
    Ke = prm.Ke;
    Ng = prm.Ng;

    // 独立変数の初期化
    x = Vector2d(0.0, 0.0);
    u = 0.0;

    target_theta = 0.0;
    theta = 0.0;
    dtheta = 0.0;
    ddtheta = 0.0;
    V = 0.0;
    I = 0.0;
    
}

void mpc::observer(const double& theta, const double& V){
    // 純粋なダイナミクス
    double dI;
    dI = (1/L)*V - (Ke/L)*dtheta - (Re/L)*I;
    I = I + dI*dt;

    ddtheta = (Kt/J)*I - (D/J)*dtheta - (Ksp*Ng/J)*theta - tau0/J;
    dtheta = dtheta + ddtheta*dt;

}

void mpc::sensorCallback(const custom_msgs::sf_to_mpc& sub_sf_mpc){
    // 現在モータ角速度を推定
    observer(theta, V);

    target_theta = sub_sf_mpc.target_angle;     // 目標モータ角度
    theta = sub_sf_mpc.angle;               // 現在モータ角度

    x[0] = theta - target_theta;   // 現在の目標角度偏差
    x[1] = dtheta - 0.0;  // 現在の目標角速度偏差

}

void mpc::control(){
    // NLPソルバー
    custom_msgs::mpc_bw_nlp server;
    // 現在状態と目標との誤差を指令値として設定
    server.request.theta_error = x[0];
    server.request.dtheta_error = x[1];
    // サービス通信をする
    if (client.call(server)){
        // 通信成功の場合、入力を受け取る
        u = server.response.input_error;
    } else {
        // 通信失敗の場合、エラー文を返す
        ROS_ERROR("Failed to call nlp server");
    }

    // 目標偏差を目標値に変換
    V = u + (Re*Ksp*Ng/Kt)*target_theta + (Re/Kt)*tau0;

}

void mpc::actuate(){
    // 現在状態をRvizに送信
    std_msgs::Float64 valve_angle;
    valve_angle.data = Ng*theta;
    pub_angle.publish(valve_angle);

    // 入力電圧をpwm生成器に送信
    if(machine_actuation){
        // pwm生成器に入力電圧を送信
        std_msgs::Float64 voltage;
        voltage.data = V;
        pub_input.publish(voltage);
    }
    
}

void mpc::spin(){
    ros::Rate loop_rate(20);
    while (ros::ok()){
        ros::spinOnce();    // sensorCallbackが呼ばれる
        control();
        actuate();

        loop_rate.sleep();
    }

}

/*timer::timer(){

}*/

// 試しになにか入力を入れてみる
// RVizでモデルを可視化してみる