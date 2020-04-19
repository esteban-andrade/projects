#include <iostream>
#include <vector>
#include <sstream>


int vector() {
    // create vector of integers called 'data'
    std::vector<int> data;

    int length = 10;
   
    // create a loop to populate elements of 'data' (each element [i] = i)
    for (int i = 0; i < length; i++) {
        data.push_back(i);
    }

    // loop to print element of vector (let's access each element location)
    for (int i = 0; i<data.size(); i++) {
        std::cout <<data.at(i) << " ";
    }
    std::cout << std::endl;

    // loop to print elements of vector (let's use auto)
    for (auto elem : data) {
        std::cout <<elem << " ";
    }
    std::cout <<std::endl;
}

int vectorofvectors () {
    // create a vector of vectors containing integers called 'data'
    std::vector<std::vector<int>> data;
    
    // create a loop to populate elements of 'data' such that is a matrix of 4x4 elements (each element row = i)
    const int rows = 4;
    const int colume = 4;
    
    for (int i=0; i<2; i++) {
    std::vector<int> row;    
        for (int j=0; j<4; j++) {
            row.push_back(i);
    }
    data.push_back(row);
    }
    
    std::cout << "Matrix" << std::endl;
    
    // loop to print elements of 'data'
    
    for (int i=0; i<data.size(); i++) {
        for (int j=0; j<data[i].size(); j++) {
        std::cout <<data.at(i).at(j) << " ";
        }
    std::cout << std::endl;
    }
}

int main () {
    vector();
    vectorofvectors();
    return 0;
}
