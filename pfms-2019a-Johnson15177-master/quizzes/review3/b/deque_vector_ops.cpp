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
      std::normal_distribution<double> distribution (mean,std_dev);
      for (int i = 0; i< num_values; i++)
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
void populateVector(std::vector<double>& values, int num_values, double mean, double std_dev) {

    // Create a random number generator and seed it from
    // the system clock so numbers are different each time
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      std::default_random_engine generator (seed);
      std::normal_distribution<double> distribution (mean,std_dev);
      for (int i = 0; i< num_values; i++)
      {
          values.push_back(distribution(generator));
      }

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

}

//! Sample function for printing elements of the deque
void printVector(std::vector<double>& values) {
    // Loop through the deque and print the values
    for (auto value : values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

void bubbleSort(std::vector<double>&buble)
{
    int n;
    bool swapped = true;
    //n = length(A)
    n = buble.size();
    //repeat
    while(swapped){
       // swapped = false
    swapped = false;
        //for i = 1 to n-1 inclusive do
    for (int i = 0; i<n-1; i++)
    {
        /* if this pair is out of order */
          //  if A[i] > A[i+1] then
        if (buble[i] > buble[i+1]){
                /* swap them and remember something changed */
            //    swap (A[i], A[i+1] )

            ///////anotherway to do it//
            //buble[i] += buble[i+1];
            //buble[i+1] = buble[i] - buble[i+1];
            //buble[i] -=buble[i+1];
            /////// /   /////////////


            std::swap(buble[i], buble[i+1]);
              //  swapped = true
           swapped = true;
        }
    }
            //end if
        //end for
    //until not swapped
//end procedure
    }
}

int main() {

    ////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers
    populateDeque(values, 10, 8.0, 4.0);

	// Print the contents of the deque
    std::cout<<"deque contains: ";
    printDeque(values);

    ////////////////////////////////////////////////

	// Create an empty vector
    std::vector<double> vector_values;

	// Populate it with random numbers
    populateVector(vector_values, 10, 8.0, 4.0);

	// Print the contents of the vector
    std::cout<<"vector contains: ";
    printVector(vector_values);
    ////////////////////////////////////////////////


    // Bubble s ///ort one of your containers
    bubbleSort(vector_values);
    // Print the contents of the container
    std::cout<<"vector buble sort: ";
    printVector(vector_values);


	return 0;
} 
