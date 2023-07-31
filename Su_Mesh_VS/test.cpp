#include <iostream>
#include <vector>
using namespace std;

class A
{
public:
    int a;
};

void tp(A *a)
{
    a->a = 1;
    return;
}

int main()
{
    vector<A> aa;
    A b;
    b.a = 0;
    aa.push_back(b);
    aa.push_back(b);
    for (std::vector<A>::iterator iter = aa.begin(); iter != aa.end(); ++iter)
    {
        tp(&(*iter));
    }
    return 0;
}