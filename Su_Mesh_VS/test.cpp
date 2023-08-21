#include <iostream>
#include <vector>
using namespace std;

int main()
{
    vector<int> AA = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    vector<int>::iterator iter;
    if ((iter = find(AA.begin(), AA.end(), 7)) == AA.end())
        cout << "1\n";
    return 0;
}
