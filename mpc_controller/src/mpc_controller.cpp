#include "mpc_controller.hpp"

void Throttle::init(){
    cout << "throttle init" << endl;
    // 変数の初期値を与える

    dt = 0.05;
    count = 0;
    controlCount = 0;
    time = 0.0;
}

void Throttle::sense(){
    if(count % controlCycle != 0) return;
    
    // ポテンショメータからの信号を受け取る
}

vector<double> Throttle::stateEq(const vector<double> x, const double U){
    // 返し値を設定するか、アンパサンドとしてdxを更新するか迷い中
    double a1, a2;
    a1 = - (Ksp/Jv);
    a2 = - (1/Jv)*(Dv + G*G*Kt*Ke/R);

    double b1, b2;
    b1 = G*Kt/(Jv*R);
    b2 = - (Tsp0/Jv);

    dx[0] = x[1];
    dx[1] = a1*x[0] + a2*x[1] + b1*U + b2;

    return dx;
}

void Throttle::control_mpc(){
    if(count % controlCycle != 0) return;

    // mpcに必要なものを書いていく
    // コスト関数
    // 制約条件
    // QPソルバー

    // 状態の更新
    stateEq(x, u);
    x = x + dx*dt;
}

bool Throttle::pwmGen(double u){

}

void Throttle::actuate(){
    if(count % controlCycle != 0) return;

    if(machineActuation){
        // 出力電圧をpwm波形に変換する
        pwmGen(u[0]);
    }
    
    // 出力信号を送る
}