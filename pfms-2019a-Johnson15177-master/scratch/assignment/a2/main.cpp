#include <iostream>
#include <string.h>

#include "rangerinterface.h"
#include "ranger.h"
#include "laser.h"
#include "radar.h"

using namespace std;

int main () {
    int input;
    Laser laser1;
    
    std::vector<Ranger*> vector;
    vector.push_back(&laser1);
    

    /* Laser Fixed Parameters */
    cout << "Laser Fixed Parameters" << endl;
    cout << "Model: " << laser1.getModel() << endl;
    
    /* Set Baud */
  //  cout << "Default Baud: " << laser1.getBaud() << endl;
  //  cout << "Set Baud (38400 or 115200): ";
  //  cin >> input;
    
  /*  if (laser1.setBaud(input) == 0)
    {
        cout << "That was not an option, using default value: " << laser1.getBaud() << endl;
    }
    cout << "Baud is set to: " << laser1.getBaud() << endl; */
    
    /* Set Angular Resolution */
 /*   cout << "Set Angular Resolution (10 or 30): " << endl;
    cin >> input;
    
    cout << "Angular Resolution is " << endl; */
    return 0;
};
