#ifndef VEHICLE_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define VEHICLE_H

#include <string>

class Vehicle {
public:
  Vehicle(std::string, std::string, std::string, int);

  std::string getMake(void);
  virtual std::string getModel(void);
  std::string getColour(void);
  int getCurrentSpeed(void);

  void setMake(std::string);
  virtual void setModel(std::string);
  void setColour(std::string);
  void setCurrentSpeed(int);

  //A Pure virtual function that should print all the general and
  //vehicle specific details
  virtual void details(void) = 0;

protected:
  std::string make_;
  std::string model_;
  std::string colour_;
  int currentSpeed_;
};

#endif // VEHICLE_H
