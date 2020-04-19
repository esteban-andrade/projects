#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console

/*
Person::Person1(std::string name, int age) {
Person::setName(name);
if(Person::setAge(age)) {
std::cout << name << "'s age is " << age <<std::endl;
} else {
    std::cout << "Something is wrong with " << name << "'s age" <<std::endl;
    }
}
*/

void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  if (age<0 || age>130) {  
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


