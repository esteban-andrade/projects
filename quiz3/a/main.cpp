#include <vector>
#include <iostream>

#include "vehicle.h"
#include "car.h"
#include "motorbike.h"

typedef std::vector<Vehicle*>::iterator it;

//Print the details of each vehicle in the vector using an iterator
void printDetails(std::vector<Vehicle*> &vehicles) {
    for(std::vector<Vehicle*>::iterator it = vehicles.begin(); it !=vehicles.end(); it++){
        (*it)->details();
    }
}

int main (void) {
    // Create a car object
    Car mustang("ford", "mustang", "red", 0, 2);

    //Create a motorbike object
    Motorbike kawasaki("kawasaki", "ninja", "blue", 0);
    //kawasaki.details();

    //Instansiating a vector of vehicles
    std::vector<Vehicle*> vehicles = {&kawasaki, &mustang};
        
    //Print the details of the vechiles
    printDetails(vehicles);

    return 0;
}
