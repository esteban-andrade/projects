#include "person.h"

#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector>

int greetMember(std::vector<Person> crowd) {
	std::cout << "\n\nI think a greeting is in order!" << std::endl;
	int masterAgeContainer = 0;
	int tempAgeContainer = 0;
	int marker = 0;
	int size = crowd.size();
	for(int i = 0; i < size; i++) {
		tempAgeContainer = crowd[i].getAge();
		if(tempAgeContainer > masterAgeContainer) {
			masterAgeContainer = tempAgeContainer;
			marker = i;
		}
	}
	std::cout << "\nHello ";
	std::cout << crowd[marker].getName();
	std::cout << ", You are our most distinguished guest!" << std::endl;
	
	return(0);
}

int main (void) {
  std::cout << "Lets create some random people" << std::endl;
  //Create the person 'alice'
  Person alice("Alice", 10);

  //Create the person 'bob' and print his age
  Person bob("Bob", 60);

  //Create the person 'carol'
  Person carol("Carole", 72);

  //! TODO Create a 'crowd' using a vector container of people
  	//First create the people to be included in the vector
  	std::cout << "\n\nLets create some friends" << std::endl;
  	Person jillian("Jillian", 21);
  	Person joseph("Joseph", 50);
  	Person hayden("Hayden", 100);
	
	//Lets fill the data for each 'person'
	std::cout << jillian.getName() << "'s age: " << jillian.getAge() << std::endl;
	std::cout << joseph.getName() << "'s age: " << joseph.getAge() << std::endl;
	std::cout << hayden.getName() << "'s age: " << hayden.getAge() << std::endl;
	
	//Lets create a vector of the class 'person' and populate it
	std::vector<Person> crowd;
	crowd.push_back(jillian);
	crowd.push_back(joseph);
	crowd.push_back(hayden);

  //! TODO Create a function that greets the oldest crowd member
  	greetMember(crowd);
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;

  
  return 0;
}

/*This code does not compile because:
	- Person header not included in the cpp file
	- Trying to access private variables directly from main
		(forbidden)
	- Not calling correct set name function
	- Input the class variable contents into the functions argument,
		let the argument fill the private variables*/
