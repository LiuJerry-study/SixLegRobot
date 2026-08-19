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
    if(m.dist <= lena + lenb&&m.dist>=lena-lenb){
        cout<<"该位置合法（在可达范围内）"<<endl;
    } else {
        cout<<"该位置超出可达范围！"<<endl;
    }
    return ;
}
void ccl(cdn& m){
    m.dist = sqrt(m.x*m.x + m.y*m.y);
    double cos1=(lena*lena+lenb*lenb-m.dist*m.dist)/(2*lena*lenb);
    double cos2=(lena*lena+m.dist*m.dist-lenb*lenb)/(2*lena*m.dist);
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

