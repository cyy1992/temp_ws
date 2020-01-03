#include <iostream>
#include <string>
using namespace std;
// class Base{
// public: 
// 	Base(int a):b(a){};
// 	~Base(){};
// 	void fun(){
// 		b= b+10;
// 	};
// 	int b;
// };
class Employee
{
public:
    Employee()
    {
        inum++;                //这里是要点： 当时没有想出来，在写一个变量来接收静态变量的变化值
        mID = inum;            //可以做到当每增加一个员工，它的 ID 会自动 加一。
        mName = "NoDefined";
        mSalary = 0.0;
    }
    
    virtual void GetPay() = 0;    //不同的计算工资方法；
    virtual void Show()            //在基类中实现出来，如果内容相同，子类就不用再实现了。当时没想到
    {
        cout << "姓名:" << mName << " ID:" << mID << " 工资:" << mSalary << endl;
    }
    virtual ~Employee(){ }            //基类的析构函数一般写成虚继承，可以做到把基类和子类对象一起析构
protected:
    int mID;
    string mName;
    double mSalary;
    int inum;            //员工ID的基数，每进一个员工，在基数上加一，就为它的ID。
};