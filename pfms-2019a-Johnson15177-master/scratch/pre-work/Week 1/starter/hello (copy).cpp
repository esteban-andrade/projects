#include <iostream>
#include <vector>
   /*
   Create a function that accepts a double value as a parameter and
Returns a square value
Returns a bool value if the double is greater than zero and the square value instead of initial passed value
Returns bool value if the double is greater than zero, the square value, the cube value and the passed value incremented by one
Loop over item 3 for 1-20
How best protect the passed value?
   */
   
   /*
   double square(double x) {
   return x*x;
}   
   
int main () {
    double input = 2.0;
    double result = 0.0; 
    result = square(input);
    std::cout << "The square of " << input << " is " << result << std::endl;
    return 0;
}
    */
    /*
bool square(double &x) {
 bool result = x > 0;
 x = x*x;
 return result;
 }
 
 int main () {
    double number = 2.0;
    double result = false; 
    result = square(number);
    std::cout << "Result of function:  " << result << std::endl;
    std::cout << "Square of number: " <<number << std::endl;
    return 0;
*/



int main () {
    std::vector<int> numbers;

    for (int i = 0; i <= 10; i++) {
        numbers.push_back(i);
    }
    for (int i = 0; i <= 10; i++) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
