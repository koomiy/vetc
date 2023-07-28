#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;
using namespace CppAD::AD;
using namespace CppAD::NearEqual;

namespace vtec{

class solveNLP   // もともとFG_eval
{
public: 
    typedef CPPAD_TESTVECTOR(AD<double>) Evector;
    solveNLP();  //コンストラクタ
    ~solveNLP(); //デストラクタ
    void operator()(Evector& fg, const Evector& x); // ()演算子のオーバーロード
    void spin();

private:
    vector<double> solve();

};

}