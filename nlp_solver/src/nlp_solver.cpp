#include "../include/nlp_solver.hpp"

using namespace std;
using namespace Eigen;
using namespace CppAD::AD;
using namespace CppAD::NearEqual;

// set the timestep length and duration
size_t N = 5;
double dt = 0.05;

// indexing
// 0~N-1まではθの値
// N~2N-1まではdθの値
// 2N~3N-1まではuの値
size_t theta_start = 0;
size_t dtheta_start = theta_start + N;
size_t u_start = dtheta_start + N;
size_t nx = (N+1)*2;
size_t nu = N*1;

////////////////////////////////////////////////
namespace vetc{

void FG_eval::operator()(Evector& fg, const Evector& x){
    // Evectorはvectorのようにpush_backコマンドを使えない
    // x のうち、どこからどこまでが x と u なのか明確にする
    // f, g の設定は計算式から見直すべき

    // 汎用インデックスの定義
    size_t i;

    // サイズが異常のとき、強制終了
    assert(fg.size() == 1 + N*2);
    assert(x.size() == N*3+2);

    // indexing
    Evector x0;
    x0 = {x[0], x[1]};
    Evector xhat;
    for (i = 2; i < (N+1)*2; i++){
        xhat.push_back(x[i]);   //x1~xN
    }
    Evector uhat;
    for (i = N*2; i < N*(2+1); i++){
        uhat.push_back(x[i]);
    }

    // 状態コスト係数の定義
    Vector2d q{1.0, 1.0};
    MatrixXd Q(2, 2);
    Q = q.asDiagonal();
    VectorXd qhat = q;
    for(i = 0; i < N-1; i++){
        qhat << qhat,
                q;
    }
    MatrixXd Qhat(N*2, N*2);
    Qhat = qhat.asDiagonal();

    // 入力コスト係数の定義
    double r = 1.0;
    VectorXd rhat = r;
    for (i = 0; i < N-1; i++){
        rhat << rhat,
                r;
    }
    MatrixXd Rhat(N, N);
    Rhat = rhat.asDiagonal();

    // 初期状態コスト
    double cost_init;
    cost_init = x0.transpose()*Q*x0;

    // 状態コスト
    double cost_state;
    cost_state = xhat.transpose()*Qhat*xhat;

    // 入力
    double cost_input;
    cost_input = uhat.transpose()*Rhat*uhat;

    // f(x)
    fg[0] = 1/2*(cost_state + cost_input + cost_init);

    // g(x)
    for (i = 1; i < (1 + N); i++){
        Evector& str;
        vector<double> vx;
        if (i == 1){
            vx[i-1] = {xhat[2*i-2], xhat[2*i-1]};
            str = vx[i-1] - (x0 + dt*stateEq(x0, uhat[i-1]));
        } else {
            vx[i-1] = {xhat[2*i-2], xhat[2*i-1]};
            vx[i-2] = {xhat[2*(i-1)-2], xhat[2*(i-1)-1]};
            str = vx[i-1] - (vx[i-2] + dt*stateEq(vx[i-2], uhat[i-1]));
        }
        
        fg[2*i - 1] = str[0];
        fg[2*i] = str[1];
    }

    return;
    
}

vector<double> solveNLP::solve(VectorXd x){
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(AD<double>) Evector;

    // 独立変数の個数
    size_t nx = N*2;

    // 制約条件の個数
    size_t ng = N*2;

    // 独立変数の初期値
    Evector xi(nx);
    for (i = 0; i < nx; i++){
        xi[i] = 0.0;
    }
    xi[0] = x[0];
    xi[1] = x[1];

    // 状態量の上限と下限を設定する
    Evector xl(nx), xu(nx);
    for (i = 0; i < nx; i++){
        // バルブ角度の制限
        xl[2*i] = 0.0;
        xu[2*i] = M_PI/2;

        // バルブ角速度の制限
        xl[2*i+1] = ;
        xu[2*i+1] = ;
    }

    // 制約関数の上限と下限を設定する
    Evector gl(ng), gu(ng);
    for (i = 0; i < ng; i++){
        // 等号制約条件
        gl[i] = 0.0;
        gu[i] = 0.0;
    }

    // 目的関数と制約条件を計算するオブジェクト
    FG_eval fg_eval;

    // NLPソルバーのオプション設定
    std::string options;

    // 処理内容をプリントしない
    options += "Integer print_level 0\n";
    options += "String  Sb          yes\n";

    // 最大反復回数
    options += "Integer max_iter    10\n";

    // 一次近似における必要精度
    options += "Numeric tol         1e-6\n";

    // 導関数のテスト??
    options += "String  derivative_test             second-order\n";

    // 有限差分評価時における、ランダム摂動量の最大値
    options += "Numeric point_perturbation_radius   0.\n";

    // オプション設定終わり

    // 解を返す変数
    CppAD::ipopt::solve_result<Evector> solution;

    // 非線形問題を解く
    CppAD::ipopt::solve<Evector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );

    // 解の一部をチェックする
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    
    auto cost = solution.obj_value;
    cout << "Cost: " << cost << endl;

    // バルブ角と角速度、および出力電圧の最適解
    vector<double> sol;
    sol.push_back(solution.x[0]);   // theta
    sol.push_back(solution.x[1]);   // dtheta
    sol.push_back(solution.x[N*2]); // U

    return sol;
}

}   // end vetc namespace