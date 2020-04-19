#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(vector<double> &numbers);

int main () {
    //TODO Create a vector of doubles with 4 values
    vector<double> vector = {1.0, 2.0, 3.0, 4.0};
    
    //TODO Add a value to the end/back
    vector.push_back(5.0);
    
    //TODO Modify the 3rd value
    vector.at(2) = 20;
    
    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword
    
        for (auto x = vector.begin(); x != vector.end(); x++) {
            std::cout << "element " << *x << std::endl;
        }
    

    //TODO Compute the sum via sum function and print sum
    std::cout << "the sum of the vector " << sum(vector) << std::endl;

    return 0;
}

// Define the sum function, the signature must match exactly
double sum(vector<double> &numbers) {
    double total = 0.0;
    //TODO Use an iterator
        for (std::vector<double>::iterator it = numbers.begin(); it != numbers.end(); ++it)
            total += *it;
    
    return total;
}
