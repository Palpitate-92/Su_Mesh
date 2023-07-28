#include <iostream>
using namespace std;

int main()
{
    int a = 0, b = 1, c = 6, d = 7;
    cout << a << ' ' << b << ' ' << c << ' ' << d << '\n';
    c = !a;
    d = !b;
    cout << a << ' ' << b << ' ' << c << ' ' << d << '\n';
    return 0;
}