#include "IMUData.h"
#include "SL_error.h"
#include <iostream>
#include <iomanip>

using namespace std;

void IMUData::print() {
    std::cout << tm << " -- " << a[0] << " " << a[1] << " " << a[2]
            << " ";
    std::cout << g[0] << " " << g[1] << " " << g[2] << std::endl;
}

void IMUData::write(std::ofstream& file) const{
    file << setprecision(24) << tm << " "
         << a[0] << " " << a[1] << " " << a[2] << " ";

    file << setprecision(12)
         << g[0] << " " << g[1] << " " << g[2] << " ";

    file << ar[0] << " " << ar[1] << " " << ar[2] << " ";
    file << gr[0] << " " << gr[1] << " " << gr[2] << endl;
}
void IMUData::write(const char* filePath) const{
    std::ofstream file(filePath);
    if (!filePath)
        repErr("%s : cannot open %s to write!", __func__, filePath);

    write(file);
    file.close();
}
