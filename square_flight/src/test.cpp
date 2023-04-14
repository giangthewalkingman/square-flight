#include <iostream>

int main() {
    int i = 0;
    double x[10], y[10], z[10];
while(1) {
    std::cout << "When i = " << i << std::endl;
        if(0 <= i && i <= 54) {
            for(int j = 0; j < 10; j++) {
                x[j] = i;
            }
        }
        if(54 < i && i < 64) {
            for(int j = 0; j < 10; j++) {
                if(i+j <= 63) {
                    y[j] = i;
                }
                if(i+j > 63) {
                x[j] = 63;
                }
            }
        }
        for(int j = 0; j < 10; j++) {
            std::cout<<x[j] <<", " <<y[j]<<", " << z[j] << std::endl;
        }
        i++;
        if(i==64) {
            break;
        }
    }
}
