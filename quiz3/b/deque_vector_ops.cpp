#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev) {

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution (mean, std_dev);

	for(unsigned int i = 0; i < num_values; i++){
		values.push_back(distribution(generator));
	}

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/
}

void populateVector(std::vector<double>& vector, int num_values, double mean, double std_dev) {

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution (mean, std_dev);

	for(unsigned int i = 0; i < num_values; i++){
		vector.push_back(distribution(generator));
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

void printVector(std::vector<double>& vector) {
	// Loop through the deque and print the values
	for (auto value : vector) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

void swap(double *xp, double *yp){
	double temp = *xp;
	*xp = *yp;
	*yp = temp;
}

void bubbleSortDeque(std::deque<double> &values, int numElements){
	for(int i = 0; i < numElements - 1; i++){
		for(int j = 0; j < numElements - i - 1; j++){
			if(values.at(j) > values.at(j+1)){
				swap(&values.at(j), &values.at(j+1));
			}
		}
	}
}

void bubbleSortVector(std::vector<double> &vector, int numElements){
	for(int i = 0; i < numElements - 1; i++){
		for(int j = 0; j < numElements - i - 1; j++){
			if(vector.at(j) > vector.at(j+1)){
				swap(&vector.at(j), &vector.at(j+1));
			}
		}
	}
}

int main() {
	int numElements;
	double mean;
	double std_dev;
    ////////////////////////////////////////////////
	std::cout << "How many elements?" << std::endl;
	std::cin >> numElements;
	std::cout << "What is the mean?" << std::endl;
	std::cin >> mean;
	std::cout << "What is the standard deviation?" << std::endl;
	std::cin >> std_dev;
	// Create an empty deque
	std::deque<double> values;

	// Populate it with random numbers
	populateDeque(values, numElements, mean, std_dev);
	// Print the contents of the deque
	printDeque(values);
    ////////////////////////////////////////////////

	// Create an empty vector
	std::vector<double> vector;
	// Populate it with random numbers
	populateVector(vector, numElements, mean, std_dev);
	// Print the contents of the vector
	printVector(vector);
    ////////////////////////////////////////////////

	int container;
	std::cout << "Press 1 to sort deque, press 2 to sort vector:" << std::endl;
	std::cin >> container;
    // Bubble sort one of your containers
	if(container == 1){
		bubbleSortDeque(values, numElements);
		printDeque(values);
	} else if(container == 2){
		bubbleSortVector(vector, numElements);
		printVector(vector);
	}
	
	// Print the contents of the container


	return 0;
} 
