#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double> &numbers);

int main () {
    //TODO Create a vector of doubles with 4 values

    //TODO Add a value to the end/back

    //TODO Modify the 3rd value

    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword 

    //TODO Compute the sum via sun function and print sum

    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers) {
    double total = 0.0;
    //TODO Use an iterator

    return total;
}
