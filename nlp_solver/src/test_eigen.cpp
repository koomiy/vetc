#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

bool eigen_array(void){
    bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;

    typedef CppAD::eigen_vector<AD<double>> a_vector;

    size_t n = 10, m = n;
    a_vector a_x(n), a_y(m);

    // a_xは1~10までの列ベクトル
    for (size_t j = 0; j < n; j++){
        a_x[j] = double(1+j);
    }
    CppAD::Independent(a_x);
    
    // a_yは以下の式で表される列ベクトル
    for (size_t j = 0; j < n; j++){
        a_y[j] = a_x[j] + sin(a_x[j]);
    }

    // ADFunとはなんだろう
    CppAD::ADFun<double> f(a_x, a_y);

    CPPAD_TESTVECTOR(double) x(n);
    for (size_t j = 0; j < n; j++){
        x[j] = double(j) + 1.0 / double(j+1);
    }
    CPPAD_TESTVECTOR(double) jac = f.jacobian(x);

    double eps = 100. * CppAD::numeric_limits<double>::epsilon();
    for(size_t i = 0; i < m; i++){
        for(size_t j = 0; j < n; j++){
            double check = 1.0 + cos(x[i]);
            if(i != j){
                check = 0.0;
            }
            ok &= NearEqual(jac[i*n + j], check, eps, eps);
        }
        
    }

    return ok;
}