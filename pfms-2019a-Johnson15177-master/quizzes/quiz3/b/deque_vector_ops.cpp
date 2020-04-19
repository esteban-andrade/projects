#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev) {
    
	// Create a random number generator and seed it from
	// the system clock so numbers are different each time
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, std_dev);
    
    
    for (int i=0; i < num_values; i++)
    {
        values.push_back(distribution(generator));
    }
    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values) {
	// Loop through the deque and print the values
	for (auto value : values) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

void bubble_sort(std::deque<double>& values)
{
    int n = values.size();
    int t;
    for (int i=0; i < n -1; i++)
    {
        for (int j=0; j < n-i-1; j++)
        {
            if(values.at(j) > values.at(j+1))
            {
                t = values.at(j);
                values.at(j) = values.at(j+1);
                values.at(j+1) = t;
            }
        }
    }
}
void populateVector(std::vector<double>& values1, int num_values, double mean, double std_dev) {
    
	// Create a random number generator and seed it from
	// the system clock so numbers are different each time
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, std_dev);
    
    
    for (int i=0; i < num_values; i++)
    {
        values1.push_back(distribution(generator));
    }
    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

}
void printVector(std::vector<double>& values1) {
	// Loop through the deque and print the values
	for (auto value : values1) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

void bubble_sort(std::vector<double>& values1)
{
    int n = values1.size();
    int t;
    for (int i=0; i < n -1; i++)
    {
        for (int j=0; j < n-i-1; j++)
        {
            if(values1.at(j) > values1.at(j+1))
            {
                t = values1.at(j);
                values1.at(j) = values1.at(j+1);
                values1.at(j+1) = t;
            }
        }
    }
}

int main() {

    ////////////////////////////////////////////////
    
	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers
    populateDeque(values, 5, 8, 4);
	// Print the contents of the deque
    printDeque(values);
    ////////////////////////////////////////////////

	// Create an empty vector
    std::vector<double> values1;
	// Populate it with random numbers
    populateVector(values1, 5, 8, 4);
	// Print the contents of the vector
    printVector(values1);
    ////////////////////////////////////////////////


    // Bubble sort one of your containers
    bubble_sort(values1);
	// Print the contents of the container
    printVector(values1);

	return 0;
} 
