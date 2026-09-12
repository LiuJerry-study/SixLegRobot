// 历史验证程序：单腿 2 自由度 IK 的控制台小工具（角度计算/robot_leg.py 与固件公式的源头）。
// 用法：依次输入 大腿长/小腿长、起始足端坐标、终止足端坐标（cm），
//       程序打印这两点的 ag1/ag2（度）并判断是否落在几何可达范围内。
// 口径（本文件保留 2026-09 之前的历史写法：acos + atan2，结果弧度、"体内侧为正"）：
//   ag1 = acos((lena²+lenb²−dist²)/(2·lena·lenb))
//   ag2 = acos((lena²+dist²−lenb²)/(2·lena·dist)) + atan2(x, y)   ← 弧度，
//         ag2 = 大腿方向相对 +y 轴的角度、偏向 +x（体内侧）为正；cos 先夹逼到 [−1,1]，
//         dist < 1e-9 直接返回；只在输出时经 to_deg() 转成度。
// ⚠ 与固件的现状对照（2026-09-10 起）：
//   固件 src/LegKinematics.cpp::ik() 现在返回的是【本文件结果的取反】——
//     ag2_fw = −( acos(...) + atan2(fx, fy) )，即"外倾为正"口径；
//   默认站姿 (−6,−10)：本程序给 ag2 ≈ −67.38°，固件给 +67.38°（同一姿态、符号相反），
//   而 ag1 两边都是 +67.38°（acos 恒为正）。
//   配套映射（2026-09 实物实测改版）：固件 src/SingleLeg.cpp 写 servoThigh = ag2_fw → 站姿
//   67.38°（Config.h::ANGLE_OFFSET 整表已清零 ⇒ 最终写入角 = 逻辑角）、servoKnee = 180 − ag1
//   → 112.62°；角度计算/robot_leg.py 已同步成固件口径。
//   ⚠ 旧写法 90 + ag2_fw（站姿 157.38°、叠 leg0 偏移后 173.38°）已被实物实测证伪：那台机器上
//     大腿会转过水平、快指到最下面，站姿摆不出来 —— 该写法已作废。
//   注：本文件在文件作用域定义 const double PI 是安全的（不 include <Arduino.h>）；固件那边
//   曾因同样的写法撞上 Arduino.h 的 #define PI 而编译失败，该行现已删除、pio run 已通过。
//   本文件保持历史写法不变，要统一口径时以 src/ 当前代码为准。
#include<iostream>
#include<cmath>
#include<iomanip>
using namespace std;
const double PI = 3.14159265358979323846;
struct cdn{//弧度制
    double x,y;
    double dist;
    double ag1;//两腿角
    double ag2;//机腿角
};
double lena,lenb;
cdn start,ed;

void check(cdn m);//先声明
void ccl(cdn& m);
double to_deg(double rad);

void iin(){
    cout<<"输入两条腿的长度(大腿先，小腿后)：";
    cin>>lena>>lenb;
    cout<<"输入脚初始位置";
    cin>>start.x>>start.y;
    start.dist = sqrt(start.x*start.x + start.y*start.y);
    check(start);
    cout<<"输入脚末位置";
    cin>>ed.x>>ed.y;
    ed.dist = sqrt(ed.x*ed.x + ed.y*ed.y);
    check(ed);
    return ;
}
void check(cdn m){
    if(m.dist <= lena + lenb && m.dist>=fabs(lena-lenb)){
        cout<<"该位置合法（在可达范围内）"<<endl;
    } else {
        cout<<"该位置超出可达范围！"<<endl;
    }
    return ;
}
void ccl(cdn& m){
    m.dist = sqrt(m.x*m.x + m.y*m.y);
    if(m.dist < 1e-9) return;   // 原点附近无法构成角度，避免除零/NaN
    double cos1=(lena*lena+lenb*lenb-m.dist*m.dist)/(2*lena*lenb);
    double cos2=(lena*lena+m.dist*m.dist-lenb*lenb)/(2*lena*m.dist);
    // 余弦夹逼到 [-1,1]，避免浮点误差使 acos 返回 NaN
    if(cos1 > 1) cos1 = 1; else if(cos1 < -1) cos1 = -1;
    if(cos2 > 1) cos2 = 1; else if(cos2 < -1) cos2 = -1;
    m.ag1=acos(cos1);
    m.ag2=acos(cos2)+atan2(m.x,m.y);
}
double to_deg(double rad){
    return rad * 180.0 / PI;
}
int main(){
    iin();
    ccl(start);
    ccl(ed);
    cout<<"两腿角："<<fixed<<setprecision(2)<<to_deg(start.ag1)<<"°"<<"->"<<to_deg(ed.ag1)<<"°"<<endl;
    cout<<"腿机角："<<fixed<<setprecision(2)<<to_deg(start.ag2)<<"°"<<"->"<<to_deg(ed.ag2)<<"°"<<endl;
    return 0;
}

