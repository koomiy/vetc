#include "throttle.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

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
namespace {
    using CppAD::AD;
    using CppAD::NearEqual;

    class FG_eval { // これはthrottle.hに記述すべきでは？
    public:
        typedef CPPAD_TESTVECTOR(AD<double>) Evector;   // eigen vectorの型定義
        void operator()(Evector& fg, const Evector& x){
            size_t i;
            assert(fg.size() == 1 + N*2);   // assertは式が偽のときにプログラムを異常終了するコマンド
            assert(x.size() == N*3+2);

            // x0をどっから持ってこようか
            // 構造上、FG_evalのクラス内にstateEqを定義せざるを得ない？
            // あるいは、FG_evalにThrottleを継承させるか
            // そうすると、Throttleで定義したxとoperatorで定義したxが干渉するから要改善
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

            // initial state cost
            double cost_init;
            cost_init = x0.transpose()*Q*x0;

            // state cost
            double cost_state;
            cost_state = xhat.transpose()*Qhat*xhat;

            // input cost
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
    };
}

// VectorXdとかvectorの互換性整備
// 最適ソルバー以外の部分の構築
vector<double> solveOpt(VectorXd x){
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Evector;

    // number of independent variables
    size_t nx = N*2;

    // number of constraints
    size_t ng = N*2;

    // initial value of the independent variables
    Evector xi(nx);
    for (i = 0; i < nx; i++){
        xi[i] = 0.0;
    }
    xi[0] = x[0];
    xi[1] = x[1];

    // lower and upper limits for x
    Evector xl(nx), xu(nx);
    for (i = 0; i < nx; i++){
        // バルブ角度の制限
        xl[2*i] = 0.0;
        xu[2*i] = pi/2;   // piをどっかで定義しておく

        // バルブ角速度の制限
        xl[2*i+1] = ;
        xu[2*i+1] = ;
    }

    // lower and upper limits for g
    Evector gl(ng), gu(ng);
    for (i = 0; i < ng; i++){
        // 等号制約条件
        gl[i] = 0.0;
        gu[i] = 0.0;
    }

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;

    // turn off any printing
    options += "Integer print_level 0\n";
    options += "String  Sb          yes\n";

    // maximum number of iterations
    options += "Integer max_iter    10\n";

    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Page 25-27, Equation (6)
    options += "Numeric tol         1e-6\n";

    // dervative testing
    options += "String  derivative_test             second-order\n";

    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";

    // place to return solution
    CppAD::ipopt::solve_result<Evector> solution;

    // solve the problem
    CppAD::ipopt::solve<Evector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );

    // Check some of the solution values
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
////////////////////////////////////////////////

Throttle::Throttle(){
    // 外部環境変数？の初期値を与える
}

/*void Throttle::read(XMLNode* node){
    // xmlファイルからthrottleに必要な定数を読み込む
}*/

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