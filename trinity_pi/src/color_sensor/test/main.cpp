#include "../TCS34725.h"

#include <iostream>

using std::cout;
using std::endl;

const char *interface = "/dev/i2c-1";

int main(){

    TCS34725 colorSensor(interface);

    float r, g, b;

    while(true){
        colorSensor.getRGB(&r, &g, &b);
        float avg = (r + g + b)/3.f;
        /*cout
            << "r[" << r << "]"
            << "g[" << g << "]"
            << "b[" << b << "]"
            << endl;

        cout << "avg:" << avg << endl;*/
    }

    return 0;
}
