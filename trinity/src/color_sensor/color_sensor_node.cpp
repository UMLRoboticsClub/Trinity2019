#include "TCS34725.h"

const char *interface = "/dev/i2c-1";

int main(){

    TCS34725 colorSensor(interface);

    return 0;
}
