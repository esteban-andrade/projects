#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <vector>


// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 100

// function to print elements of the array
void printArray(double* x, int xSize){
  for (double *ip = x; ip<(x+xSize); *ip++){
      std::cout << "*ip = " << *ip << std::endl;
  }
}

// function to print elements of the array
void printArray2(double x[], int xSize){
  for (int i = 0; i<xSize; i++) {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }
}



// function to populate array with random numbers
void populateWithRandomNumbers(double num_array[], int& array_size, int num_elements) {

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0,10.0);
    // generate the required amount of random numbers
    for (int i=array_size; i<array_size+num_elements; i++) {
        num_array[i] = value_distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}

// function that prints only the elements larger than : mean + one standard deviation 4.5 + 2.87, >7
void printele(double x[], int xSize) {
    for (int i = 7; i<xSize; i++) {
    std::cout << "The elements printed are "<< x[i] <<std::endl;
    }
}

// Create a function that assigns elements of array x to a vector named *vec* (HINT: decide on the correct type)

int Vector() {
    std::vector<int> vec;
    for(int i = 0; i < 10; i++) {
       vec.push_back(i);
    }
    for (int i = 0; i < vec.size(); i++) {
        std::cout << vec.at(i) << " ";
    }
    std::cout << std::endl;
}


// Every executable needs a main function which returns an int
int main () {

Vector();

    // Create an array x of doubles with 10 elements
    int arraySize=10;
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[arraySize] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

  //  printele(x, arraySize);


    // Print array
    printArray2(x,arraySize);

    // Ask user to specify how many additional numbers are to be generated
    int num;
    std::cout << "How many random elements do you wish to generate : ";
     while(!(std::cin >> num)){
         std::cin.clear();
         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
         std::cout << "Invalid input, Try again: ";
     }

    // Populate array with random numbers
    populateWithRandomNumbers(x,arraySize,num);


    // Print array
    printArray(x,arraySize);


    // Main function should return an integer
    return 0;
}
