#ifndef PERSON_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define PERSON_H

#include <string>

class Person {
public:
  Person();
  Person(std::string name, int age){
    name_ = name;
    age_ = age;
  };
  void setName(std::string);
  bool setAge(int);

  std::string getName(void);
  int getAge(void);

private:
  std::string name_;
  int age_;
};


#endif // PERSON_H
