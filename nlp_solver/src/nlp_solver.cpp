#include "../include/nlp_solver.hpp"

using namespace std;
using namespace Eigen;
using CppAD::AD;

void FG_eval::operator()(ADvector& fg, const ADvector& x){
    // サイズが異常のとき、強制終了
    assert(fg.size() == 1 + 2*N); // 目的関数で1、制約関数で2*N
    assert(x.size() == 2*N + N);  // 予測状態軌跡で2*N、将来入力軌跡で2*N

    // 状態コストQ, 終端コストH, 入力コストR
    q << 1.0, 1.0;
    h << 10.0, 10.0;
    R = 1.0;
    Q = q.asDiagonal();
    H = h.asDiagonal();

    // Qhat, Rhat
    Qhat = Matrix<AD<double>, 2*N, 2*N>::Zero(2*N, 2*N); // 行列の部分を参照して、そこに入れたい行列を代入するってな操作
    for (int i = 0; i < N; i++){
        if(i == N-1) Qhat.block(2*i,2*i, 2,2) = H;
        else Qhat.block(2*i,2*i, 2,2) = Q;
    }
    Matrix<AD<double>, N, 1> rhat;
    for (int i = 0; i < N; i++){
        rhat[i] = R;
    }
    Rhat = rhat.asDiagonal();

    // 状態方程式行列A, B
    A << 0.0            , 1.0                  , 
         -(1/J)*(Ng*Ksp), -(1/J)*(D + Ke*Kt/Re);
    B << 0.0, Kt/(Re*J);

    // Ahat, Bhat
    Ahat = Matrix<AD<double>, 2*N, 2>::Zero(2*N, 2);
    for (int i = 0; i < N; i++){
        if (i == 0) {
            Ahat.block(2*i,0, 2,2) = A;
        } else {
            Ahat.block(2*i,0, 2,2) = Ahat.block(2*(i-1),0, 2,2)*A;
        }
    }
    Bhat = Matrix<AD<double>, 2*N, N>::Zero(2*N, N);
    for (int i = 0; i < N; i++){    // 要検算
        if(i == 0){
            for (int j = 0; j < N-i; j++){
                Bhat.block(2*i,0, 2*(N-i),(N-i)).block(2*j,j, 2,1) = B;
            }
        } else {
            for (int j = 0; j < N-i; j++){
                Bhat.block(2*i,0, 2*(N-i),(N-i)).block(2*j,j, 2,1) = A*Bhat.block(2*(i-1),0, 2, 1);
            }
        }
    }

    // 目的関数行列F、制約関数行列G
    F = Matrix<AD<double>, 3*N, 3*N>::Zero(3*N, 3*N);
    F.block(0,0, 2*N,2*N) = Qhat;
    F.block(2*N,2*N, N,N) = Rhat;
    G = Matrix<AD<double>, 2*N, 3*N>::Zero(2*N, 3*N);
    Matrix<AD<double>, 2*N, 2*N> I = Matrix<AD<double>, 2*N, 2*N>::Identity(2*N, 2*N);
    G.block(0,0, 2*N,2*N) = I;
    G.block(0,2*N, 2*N,N) = -Bhat;

    // 目的関数f(x)、制約関数g(x)
    fg[0] = (x.transpose()*(F*x) + xk.transpose()*(Q*xk)).value();  // f(x) ちゃんと値が計算されているか要確認
    Matrix<AD<double>, 2*N, 1> g;
    g = G*x - Ahat*xk;
    for (int i = 0; i < 2*N; i++){
        fg[i+1] = g[i]; // g(x) ちゃんと値が計算されているか要確認
    }

    return;
    
}

solveNLP::solveNLP() : 
    success(true), 
    //N(PREDICTION_STEPS_NUM), 
    dt(0.05), 
    nx(2*N + N), 
    ng(2*N)
{   
    // サービスサーバーの設定
    server = nh.advertiseService("solve", &solveNLP::mpcService, this);   // 引数はこれだけでいいのか？

    // 独立変数の初期化
    xi.resize(nx);
    for (int i = 0; i < nx; i++){    xi[i] = 0.0;}

    // 独立変数の上限、下限を設定
    xl.resize(nx);
    xu.resize(nx);
    for (int i = 0; i < 2*N; i++){
        // モーター角度の制限
        if (i%2 == 0){
            xl[i] = 0.0;
            xu[i] = (1/Ng)*(M_PI/2);
        }

        // モーター角速度の制限(めっちゃ適当)
        else {
            xl[i] = -10.0;
            xu[i] = 10.0;
        }
    }
    for (int i = 2*N; i < nx; i++){
        // 入力電圧の制限
        xl[i] = 0.0;
        xu[i] = 12.0;
    }

    // 制約関数の上限と下限を設定
    gl.resize(ng);
    gu.resize(ng);
    for (int i = 0; i < ng; i++){
        // 等号制約
        gl[i] = 0.0;
        gu[i] = 0.0;
    }

}

bool solveNLP::mpcService(custom_msgs::mpc_bw_nlp::Request& req_mpc_states,
                          custom_msgs::mpc_bw_nlp::Response& res_nlp_input)
{
    // 現在の目標誤差を受け取る
    xk[0] = req_mpc_states.theta_error;
    xk[1] = req_mpc_states.dtheta_error;
    
    // 目的関数、制約関数の設定
    FG_eval fg_eval(xk);

    // ========== NLPソルバーのオプション設定 ========== //
    // 処理内容をプリントしない
    options += "Integer print_level 0\n";
    options += "String  sb          yes\n";

    // 最大反復回数
    options += "Integer max_iter    10\n";

    // 一次近似における必要精度
    options += "Numeric tol         1e-6\n";

    // 導関数のテスト??
    options += "String  derivative_test             second-order\n";

    // 有限差分評価時における、ランダム摂動量の最大値
    options += "Numeric point_perturbation_radius   0.\n";
    // ================================================== //

    // 非線形問題を解く
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );

    // 解の一部の整合性評価
    success &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //if (!success) return false;

    // コストのプリントアウト
    auto cost = solution.obj_value;
    cout << "Cost: " << cost << endl;

    // モーター角度・角速度、および入力電圧の最適解
    sol.clear();
    sol.resize(3);
    sol.push_back(solution.x[0]);   // θm
    sol.push_back(solution.x[1]);   // dθm
    sol.push_back(solution.x[2*N]); // V

    // 入力電圧の送信
    res_nlp_input.input_error = sol[2];

    return true;

}

void solveNLP::spin(){
    ros::Rate loop_rate(20);    // Hz
    while (ros::ok()){  //  && success
        // Callback関数の呼び出し
        ros::spinOnce();
        loop_rate.sleep();
    }

}
