#include "assignment.h"

//initialising the random generator and setting the minimum value to 0 and max value to 255
HyperCamera::HyperCamera(int seed, int n):
    generator_(seed), distribution_(0.0, 255.0), Max_Amount_(n)
{
}

void HyperCamera::RandNumTo(std::vector<double> &num_vec1) {
// creating a vector of vector of vector called matrix
std::vector<std::vector<std::vector<double>>> matrix;
    
// gets input for sampling rate
    int input;
    std::cin >> input;
// switch to change between 1Hz and 2Hz
    switch (input) {
        case 1: 
// populates the first layer with random numbers in while loop to continuously print
    while (1) {
    for (int i=0; i<1; i++) {
    std::vector<std::vector<double>> row;
        for (int j=0; j<2; j++) {
        std::vector<double> column;        
            for (int k=0; k<4; k++) {
            column.push_back(distribution_(generator_));
            }
            row.push_back(column);
        }
        matrix.push_back(row);
    }
// populates the rest of the layers *0.8 
    for (int i=1; i<8; i++) {
    std::vector<std::vector<double>> row;
        for (int j=0; j<2; j++) {
        std::vector<double> column;
            for (int k=0; k<4; k++) {
            column.push_back((matrix.at(i-1).at(j).at(k))*0.8); 
            }
            row.push_back(column);
        }
        matrix.push_back(row);
    }
// prints the elements in matrix to the sceen
    std::cout << "Sampling rate 1Hz" << std::endl;
    for (int i=0; i<matrix.size();i++) {
        for (int j=0; j<matrix[i].size(); j++) {            
            for (int k=0; k<matrix[i][j].size(); k++) {
            std::cout << matrix.at(i).at(j).at(k) << " ";
            }
        std::cout << "\n";
        }
        std::cout << "\n";  
    }
// makes the printing of matrix occur every 1 secound
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout<<std::endl; 
    matrix.clear();
    }
        break;
    
        case 2:
    while (1) {
    for (int i=0; i<1; i++) {
    std::vector<std::vector<double>> row;
        for (int j=0; j<2; j++) {
        std::vector<double> column;        
            for (int k=0; k<4; k++) {
            column.push_back(distribution_(generator_));
            }
            row.push_back(column);
        }
        matrix.push_back(row);
    }
    
    for (int i=1; i<8; i++) {
    std::vector<std::vector<double>> row;
        for (int j=0; j<2; j++) {
        std::vector<double> column;
            for (int k=0; k<4; k++) {
            column.push_back((matrix.at(i-1).at(j).at(k))*0.8); 
            }
            row.push_back(column);
        }
        matrix.push_back(row);
    }
    
    std::cout << "Sampling rate 2Hz" << std::endl;
    for (int i=0; i<matrix.size();i++) {
        for (int j=0; j<matrix[i].size(); j++) {            
            for (int k=0; k<matrix[i][j].size(); k++) {
            std::cout << matrix.at(i).at(j).at(k) << " ";
            }
        std::cout << "\n";
        }
        std::cout << "\n";  
    }
// prints elements of matrix 2 times every second
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout<<std::endl; 
    matrix.clear();
    }
        break;
// prints to screen when input other than 1 or 2 is pressed        
        default: std::cout << "Did not enter 1 or 2" << std::endl;
    } 
}

int main()
{
    int input;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    int Max_Amount_;
    std::cout << "Enter in the sampling rate (1Hz or 2Hz)" << std::endl;
    HyperCamera filler(seed, Max_Amount_);
    
    std::vector<double> numbers;
    filler.RandNumTo(numbers);
       
    return 0;
}

























