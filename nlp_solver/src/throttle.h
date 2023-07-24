#pragma once
#include <vector>

using namespace std;

class Throttle{
public:
    // 変数の定義
    vector<double> x;
    vector<double> dx;
    vector<double> u;

    double dt;
    int count;
    int controlCount;
    double time;

    int controlCycle;
    int saveCycle;

    bool machineActuation;

public:
    // virtual void Read(XMLNode* node); xmlファイルから定数を読み込む
    virtual void init();

    virtual void sense();
    virtual void control_mpc();
    virtual void actuate();

    vector<double> stateEq(const vector<double> x, const double u);
    bool pwmGen(double u);

    Throttle();
};