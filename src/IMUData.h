#ifndef IMUDATA_H
#define IMUDATA_H

#include <fstream>

typedef unsigned short int uint16_t;
typedef short int int16_t;


class IMUData {
public:
    double tm; //time stamp
    double a[3];
    double g[3];
    uint16_t ar[3]; //raw measurements of accelerators
    int16_t gr[3];  //raw measurements of gyroscopes

public:
    IMUData(){}
    IMUData(const IMUData& other){
        tm = other.tm;
        a[0] = other.a[0];
        a[1] = other.a[1];
        a[2] = other.a[2];

        g[0] = other.g[0];
        g[1] = other.g[1];
        g[2] = other.g[2];

        ar[0] = other.ar[0];
        ar[1] = other.ar[1];
        ar[2] = other.ar[2];

        gr[0] = other.gr[0];
        gr[1] = other.gr[1];
        gr[2] = other.gr[2];
    }

    IMUData& operator=(const IMUData& other) {
        if (&other != this) {
            tm = other.tm;
            a[0] = other.a[0];
            a[1] = other.a[1];
            a[2] = other.a[2];

            g[0] = other.g[0];
            g[1] = other.g[1];
            g[2] = other.g[2];

            ar[0] = other.ar[0];
            ar[1] = other.ar[1];
            ar[2] = other.ar[2];

            gr[0] = other.gr[0];
            gr[1] = other.gr[1];
            gr[2] = other.gr[2];
        }
        return *this;
    }

    void print();
    void write(std::ofstream& file) const;
    void write(const char* filePath) const;
};

#endif // IMUDATA_H
