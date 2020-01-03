#include "base.h"

// class Sub1:public Base {
// public:
// 	Sub1(int a):c(a){};
// 	~Sub1(){};
// 	int c;
// };

class Manager : virtual public Employee
{
public:
    Manager(){}
    Manager(string name)
    {
        mName = name;
        base = 8000;
    }
    
    virtual void GetPay()
    {
        mSalary = base;
    }
    
protected:
    double base;
};