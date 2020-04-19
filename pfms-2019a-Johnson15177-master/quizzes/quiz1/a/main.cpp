#include <iostream> // Includes std::cout and friends so we can output to console
#include "person.h"
#include <vector>


int main (void) {
  //Create the person 'alice'
 // Person Alice("Alice", 32);
  Person alice;
  alice.name_ = "Alice";
  alice.age_ = 32;
 
  //Create the person 'bob' and print his age
 // Person Bob("Bob", 62);
  Person bob;
  bob.name_ = "Bob";
  bob.age_ = 62;
  //Create the person 'carol'
 // Person Carol ("Carol", 72);
  Person carol;
  carol.name_ = "Carol";
  carol.age_ = 72;

  //! TODO Create a 'crowd' using a vector container of people
    std::vector<std::string> crowd;
    std::cout << "The vector container of people" << std::endl;
    
    for (int i=0; i<1; i++) {
    crowd.push_back(bob.name_);
    crowd.push_back(alice.name_);
    crowd.push_back(carol.name_);
    }
    for (int i=0; i<crowd.size(); i++) {
    std::cout << crowd.at(i) << " ";
    }    
   std::cout<<std::endl; 
   
  //! TODO Create a function that greets the oldest crowd member
  if (alice.age_>bob.age_) {    
    //alice is greater than bob
    if (alice.age_>carol.age_) {
        //alice is greatest age
        std::cout << "Hello Alice, you're our most distinguished member" << std::endl;
    } else {
    //carol is greatest age
    std::cout << "Hello Carol, you're our most distinguished member" << std::endl;
    }
  } else if (bob.age_>alice.age_) {
  // bob is greater than alice
    if (bob.age_> carol.age_) {
    //bob is greatest age
    std::cout << "Hello Bob, you're our most distinguished member" << std::endl;
    } else {
        // carrol is greatest age
        std::cout << "Hello Carol, you're our most distinguished member" << std::endl;
    }
  }
  
      
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;


  return 0;
}
