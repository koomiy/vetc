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
    Re = prm.Re;
    Ke = prm.Ke;
    Ng = prm.Ng;

    // 独立変数の初期化
    x = Vector2d(0.0, 0.0);
    dx = Vector2d(0.0, 0.0);
    cur_angle = 0.0;
    target_angle = 0.0;
    u = 0.0;
    
}

void mpc::sensorCallback(const custom_msgs::sf_to_mpc& sub_sf_mpc){
    //if(count % controlCycle != 0) return;
    
    // ポテンショメータからの信号を受け取る
    // バルブ側から現在の状態を、アクセルペダル側から目標状態を受け取る
    // 現在状態の目標誤差をMPCの状態として定義する
    double x_pre = x[0];
    target_angle = sub_sf_mpc.target_angle;
    double x_cur = sub_sf_mpc.angle - target_angle;
    x[0] = x_cur;
    x[1] = (x_cur - x_pre)/dt;

}

void mpc::stateEq(const Vector2d& x, const double& u){
    A << 0.0            , 1.0                  , 
         -(1/J)*(Ng*Ksp), -(1/J)*(D + Ke*Kt/Re);
    B << 0.0, Kt/(Re*J);

    dx = A*x + B*u;

}

void mpc::control(){
    //if(count % controlCycle != 0) return;

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

    // 状態の更新
    stateEq(x, u);
    x += dx*dt;

}

void mpc::actuate(){
    //if(count % controlCycle != 0) return;

    // 現在状態をRvizに送信
    cur_angle = x[0] + target_angle;
    std_msgs::Float64 valve_angle;
    valve_angle.data = Ng*(cur_angle);
    pub_angle.publish(valve_angle);

    // 入力電圧をpwm生成器に送信
    if(machine_actuation){
        // 出力電圧をpwm波形に変換する
        std_msgs::Float64 V;
        V.data += u;
        pub_input.publish(V);
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