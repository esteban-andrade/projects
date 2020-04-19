#include "person.h"

#include <string>
#include <iostream> // Includes std::cout and friends so we can output to console

Person::Person(std::string passName, int passAge) {
  Person::setName(passName);
  if (Person::setAge(passAge)) {
    std::cout << passName << "'s age is " << passAge << std::endl;
  } else {
    std::cout << "Something is wrong with " << passName << "'s age" << std::endl;
  }
}

void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  if(age < 0 || age > 130) {
  	return false;
  } else {
    age_ = age;
    return true;
  }
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}
