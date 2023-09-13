#include <Vector>
#include <iostream>
using namespace std;
int main(){
    vector<int> v(26, 0);
    for(auto i: v){
        cout << i << " ";
    }
    cout << "\n";
}