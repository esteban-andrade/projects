#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console

Person::Person(){
}


void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  age_ = age;
  if (age_ < 0){
    std::cout << "Input proper age of " << name_  << ", they have a value of "<< age_ << std::endl;
    std::cin >> age_;
    return true;
  }
  return true;
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}
