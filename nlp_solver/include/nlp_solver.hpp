#pragma once

#include "cppad/cppad.hpp"
#include "cppad/ipopt/solve.hpp"
#include "cppad/example/cppad_eigen.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace CppAD::AD;
using namespace CppAD::NearEqual;

namespace vtec{

class FG_evel
{
public: 
    typedef CPPAD_TESTVECTOR(AD<double>) Evector;   // Evectorを、Eigen::Matrix<AD<double>, Eigen::Dynamic, 1>(要素がAD<double>型の可変長eigenベクトル)として型定義
    void operator()(Evector& fg, const Evector& x); // ()演算子のオーバーロード

};

class solveNLP
{
public: 
    solveNLP();
    ~solveNLP();
    void spin();

private: 
    vector<double> solve(VectorXd x);

};

}