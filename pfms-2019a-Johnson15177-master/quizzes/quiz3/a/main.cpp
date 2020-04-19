#include <vector>
#include <iostream>

#include "vehicle.h"
#include "car.h"
#include "motorbike.h"

using namespace std;

//Print the details of each vehicle in the vector using an iterator
void printDetails(vector<Vehicle*> &vehicles) {
    for (vector<Vehicle*>::iterator it = vehicles.begin(); it != vehicles.end(); ++it)
    {
        (*it)-> details();
    }
}

int main (void) {
    // Create a car object
    Car mustang("ford", "mustang", "red", 0, 2);

    //Create a motorbike object
    Motorbike kawasaki("kawasaki", "ninja", "blue", 0);

    //Instansiating a vector of vehicles
    vector<Vehicle*> vehicles;
    vehicles.push_back(&mustang);
    vehicles.push_back(&kawasaki);
    //Print the details of the vechiles
    
    printDetails(vehicles);
  
    return 0;
}
