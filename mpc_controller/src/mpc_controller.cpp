#include "../include/mpc_controller.hpp"

mpc::mpc() : 
    count(0), 
    controlCycle(1), 
    dt(0.05), 
    time(0.0)
{
    // サブスクライバのハンドラ立ち上げ
    sub_angles = nh.subscribe("/sf_to_mpc", 10, &mpc::sensorCallback, this);
    sub_input = nh.subscribe("/nlp_to_mpc", 10, &mpc::nlpCallback, this);

    // パブリッシャのハンドラ立ち上げ
    pub_curstate = nh.advertise<custom_msgs::mpc_to_nlp>("/mpc_to_nlp", 10);
    pub_input = nh.advertise<std_msgs::Float64>("/mpc_to_pwm", 10);

}

void mpc::init(){
    // パラメータ設定
    param prm;
    J = prm.J;
    D = prm.D;
    Kt = prm.Kt;
    Ksp = prm.Ksp;
    Re = prm.Re;
    Ke = prm.Ke;
    Ng = prm.Ng;

    // 独立変数の初期化
    x = Vector2d(0.0, 0.0);
    dx = Vector2d(0.0, 0.0);
    u = 0.0;
    
}

void mpc::sensorCallback(const custom_msgs::sf_to_mpc& sub_sf_mpc){
    if(count % controlCycle != 0) return;
    
    // ポテンショメータからの信号を受け取る
    double theta_past = x[0];
    double theta_cur = sub_sf_mpc.angle - sub_sf_mpc.target_angle;
    x[0] = theta_cur;
    x[1] = (theta_cur - theta_past)/dt;

}

void mpc::nlpCallback(const std_msgs::Float64& sub_nlp_mpc){
    if(count % controlCycle != 0) return;

    // 目標入力エラーを受け取る
    u = sub_nlp_mpc.data;

}

void mpc::stateEq(const Vector2d& x, const double& u){
    A << 0.0            , 1.0                  , 
         -(1/J)*(Ng*Ksp), -(1/J)*(D + Ke*Kt/Re);
    B << 0.0, Kt/(Re*J);

    dx = A*x + B*u;

}

void mpc::control(){
    if(count % controlCycle != 0) return;

    // NLPソルバー
    pub_curstate.publish(x);
    nlpCallback(sub_input);

    // 状態の更新
    stateEq(x, u);
    x += dx*dt;

}

void mpc::actuate(){
    if(count % controlCycle != 0) return;

    // 入力エラーの積分値をpwmに送信
    // 状態x[0]の積分値をrvizに送信
    if(MACHINE_ACTUATION){
        // 出力電圧をpwm波形に変換する
        std_msgs::Float64 V;
        V.data += u;
        pub_input.publish(V);
    }
    
}

void mpc::spin(){
    ros::Rate loop_rate(20);
    while (ros::ok()){
        sensorCallback(sub_angles);
        control();
        actuate();

        loop_rate.sleep();
    }

}

timer::timer(){

}