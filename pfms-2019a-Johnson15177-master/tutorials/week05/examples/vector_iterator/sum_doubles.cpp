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
    // Create a vector of doubles with 4 values
    vector<double> numbers = {2.0, 5.0, -2.0, 42.5};
    // Add a value to the end/back
    numbers.push_back(77.1);
    // Modify the 3rd value
    // Using .at(i) is safer than [i]
    numbers.at(2) = 7.0;

    // Print out the numbers
    cout << "numbers = ";
    // Range-based for loop with the auto keyword is the easiest way
    // to loop through a vector
    for (auto number : numbers) {
        cout << number << " ";
    }
    cout << endl;

    // Compute the sum and print its
    cout << "Total is " << sum(numbers) << endl;

    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers) {
    double total = 0.0;
    // Note we need a const_iterator to work with a const vector
    for (vector<double>::const_iterator number = numbers.begin();
         number != numbers.end(); number++) {
        // Need to dereference the iterator `number` with * (asterisk) to use its value
        total += *number;
    }
    return total;
}
