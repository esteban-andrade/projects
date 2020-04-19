#include <iostream> // Includes std::cout and friends so we can output to console
#include "person.h" // Includes the Person header file
#include <vector>

int getOldAge(std::vector<Person> crowd){
  int oldAge = 0;
  for(int i = 0; i < crowd.size();i++){
    if(oldAge < crowd[i].getAge()){
      oldAge = crowd[i].getAge();
    }
  }
  return oldAge;
}

std::string getOldName(std::vector<Person> crowd){
  int oldAge = 0;
  std::string oldName;
  for(int i = 0; i < crowd.size();i++){
    if(oldAge < crowd[i].getAge()){
      oldAge = crowd[i].getAge();
      oldName = crowd[i].getName();
    }
  }
  return oldName;
}

int main(void)
{
  //Create the person 'alice'
  Person alice;
  alice.setName("Alice");
  alice.setAge(32);

  //Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");

  if (bob.setAge(72))
  {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  }
  else
  {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowd;
  crowd.push_back(carol);
  crowd.push_back(bob);
  crowd.push_back(alice);

  //Prints the contents of the vector crowd
  for (int i = 0; i < crowd.size(); i++)
  {
    std::cout << crowd[i].getName() <<  " is " << crowd[i].getAge() << " years old" << std::endl;
  }

  //! TODO Create a function that greets the oldest crowd member
  std::cout << getOldName(crowd) << " is the oldest with the age " << getOldAge(crowd) << std::endl;
  //!
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;

  return 0;
}
