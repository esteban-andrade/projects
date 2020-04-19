#ifndef RANDOMVECTORFILLER_H
#define RANDOMVECTORFILLER_H
#include <vector>

// Declaration of the rectangle class
class RandomVectorFiller {

// Public members are accessible from outside the class (ie. in main)
public:
    // Declare the constructor
    RandomVectorFiller (int seed, int n):
    void appendRandomNumbersTo(std::vector<double> &num_vec);
// Private members are only accessible from within methods of the same class
private:
    int seed_;
    int count_to_append_;

};

#endif // RANDOMVECTORFILLER_H
