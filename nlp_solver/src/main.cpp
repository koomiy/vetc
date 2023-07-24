#include "ros/ros.h"
#include "throttle.h"

using namespace std;

const char* confFilename = "../conf/Visualization.conf.xml"

class VETC{
public:
    Throttle* throttle;

public:
    virtual bool initialize(){
        // 値の読み込み　できればxmlファイルから
        // 関連コードの初期化

        // 初期化とハンドル取得
        ros::init(argc, argv, "controller");  // controlに名称変更かな...
        ros::NodeHandle nh;

        throttle->init();

        return true;
    }

    virtual bool run(){
        // 時刻管理どうやってやろうか
        // 動作開始時刻を0sとして、時刻刻み幅 dt ごとに時刻を更新すればよい
        // どうやって時刻通りに出力しようか、制御器と動作器の同期問題
        // というかそもそもこれをループするっていうのどうやって設定しよう
        
        throttle->sense();
        throttle->control_mpc();    // mpcにより制御値を返す
        throttle->actuate();    // 当分はRvizに描画する機能も書いておく

        return true;
    }
};