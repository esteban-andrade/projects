#include <vector>
#include <iostream>

#include "vehicle.h"
#include "car.h"
#include "motorbike.h"
#include <vector>

//Print the details of each vehicle in the vector using an iterator
void printDetails(std::vector<Vehicle*> vehicles) {
    for(std::vector<Vehicle*>::iterator type = vehicles.begin(); type != vehicles.end(); type++)
    {
        Vehicle* typeVehicle = *type;

           typeVehicle->details();
    }
}

int main (void) {
    // Create a car object
    Car mustang("ford", "mustang", "red", 0, 2);

    //Create a motorbike object
    Motorbike kawasaki("kawasaki", "ninja", "blue", 0);


    //creating vechicle objects
    Vehicle vehicle_mustang("ford", "mustang", "red", 0);
    Vehicle vehicle_kawasaki("kawasaki", "ninja", "blue", 0);

    //Instansiating a vector of vehicles

    //creating a vector of pointers to type vehicles
    std::vector<Vehicle*> vehicles;
    vehicles.push_back(&mustang);
    vehicles.push_back(&kawasaki);

    //creating a vector of vechicles
    std::vector<Vehicle>vehicle;
    vehicle.push_back(vehicle_mustang);
    vehicle.push_back(vehicle_kawasaki);


    //Print the details of the vechiles
/*
    for(std::vector<Vehicle>::iterator type = vehicle.begin(); type != vehicle.end(); type++)
    {
       std::cout<<type->getMake()<< " "<<type->getModel()<<" "<<type->getColour()<<" "<<type->getCurrentSpeed()<<" ";
       std::cout<<std::endl;
    }

*/
    //printDetails();
    printDetails(vehicles);

    return 0;
}
