#pragma once

class Encoder {
    public:
        Encoder(unsigned pinA, unsigned pinB);
        ~Encoder();

        long count = 0;

    private:
        unsigned pinA, pinB;
        int callbackA, callbackB;
};


