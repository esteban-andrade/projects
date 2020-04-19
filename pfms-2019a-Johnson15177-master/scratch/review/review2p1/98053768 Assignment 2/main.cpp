#include <chrono>
#include <iomanip> // Lets us format our output, incl. setw(x), left, right, etc.
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "laser.h"
#include "radar.h"
#include "rangerfusion.h"

using std::cout;
using std::cin;
using std::endl;
using std::setw ;
using std::left;
using std::right;
using std::fixed;
using std::setprecision;
using std::string;
using std::vector;
using std::numeric_limits;
using std::streamsize;

int main () {
    //--------------------------------------------SETUP--------------------------------------------//
    // Tell user that the laser is being initialised
    cout << "-------------Initialisation commencing--------------" << endl;
    cout << endl;

    // Initialise sensor objects
    Laser laser;
    Radar radar1;
    Radar radar2;

    // Initialise vectors for ranger object pointers
    vector<Ranger*> rangers1;

    // Reserve space for 3 elements (1 laser, 2 radars)
    rangers1.reserve(3);

    // Put sensors into vector
    rangers1.push_back(&laser);
    rangers1.push_back(&radar1);
    rangers1.push_back(&radar2);



    // Display default values for each sensor
    cout << "Default sensor parameters: " << endl;
    cout << endl;
    for (auto  r : rangers1) {
        cout << "Model: " << r->getModel() << endl;
        cout << "Field of view: " << r->getFieldOfView() << " deg" << endl;
        cout << "Angular resolution: " << r->getAngularResolution() << " deg" << endl;
        cout << "Angle offset from laser " << r->getOffset() << " deg" << endl;
        cout << "Minimum distance: " << r->getMinRange() << " m" << endl;
        cout << "Maximum distance: " << r->getMaxRange() << " m" << endl;
        cout << endl;
    }



    int ang_res;        // Stores user's inputted angular resolution
    int offset;         // Stores user's inputted offsets
    bool input_valid;   // Stores result from sensor checking if inputted value is valid
    int fusion_method;   // Stores user's inputted fusion method selection



    // Ask user to specify angular resolution of the laser
    cout << "Enter desired  angular resolution (10 or 30 degrees) : ";
    while(!(cin >> ang_res)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    // Checks if input was valid and informs user
    input_valid = laser.setAngularResolution(ang_res);

    if (input_valid) {
        cout << "Angular resolution input valid" << endl;
        cout << "Current setting: " << laser.getAngularResolution() << " degrees" << endl;
    }
    else {
        cout << "Angular resolution input invalid" << endl;
        cout << "Setting value to default of " << laser.getAngularResolution() << " degrees" << endl;
    }
    cout << endl;



    // Ask user to specify offsets for first radar
    cout << "Enter desired offset of radar 1 from the laser (-120 to 120 degrees) : ";
    while(!(cin >> offset)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    // Checks if input was valid and informs user
    input_valid = radar1.setOffset(offset);

    if (input_valid) {
        cout << "Offset input valid" << endl;
        cout << "Current setting: " << radar1.getOffset() << " degrees" << endl;
    }
    else {
        cout << "Offset input invalid" << endl;
        cout << "Setting value to default of " << radar1.getOffset() << " degrees" << endl;
    }
    cout << endl;



    // Ask user to specify offsets for second radar
    cout << "Enter desired offset of radar 2 from the laser (-120 to 120 degrees) : ";
    while(!(cin >> offset)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    // Checks if input was valid and informs user
    input_valid = radar2.setOffset(offset);

    if (input_valid) {
        cout << "Offset input valid" << endl;
        cout << "Current setting: " << radar2.getOffset() << " degrees" << endl;
    }
    else {
        cout << "Offset input invalid" << endl;
        cout << "Setting value to default of " << radar2.getOffset() << " degrees" << endl;
    }


    // Initialise vector for RangerInterface object pointers (so it can be input into the RangerFusion class)
    vector<RangerInterface*> rangers2;


    // Reserve space for 3 elements (1 laser, 2 radars)
    rangers2.reserve(3);

    // Put pointes to same sensors into vector
    rangers2.push_back(&laser);
    rangers2.push_back(&radar1);
    rangers2.push_back(&radar2);

    // Create ranger fusion object
    RangerFusion fuser;

    cout << endl;
    cout << "Current fusion method is: ";
    switch (fuser.getFusionMethod()) {
        case FUSION_MIN:
            cout << "Minimum value" << endl;
            break;
        case FUSION_MAX:
            cout << "Maximum value" << endl;
            break;
        case FUSION_AVG:
            cout << "Average value" << endl;
            break;
    }
    cout << endl;

    // Ask user to specify fusion type
    cout << "Enter desired fusion type." << endl;
    cout << "Options are: " << endl;
    cout << "1. Minimum value" << endl;
    cout << "2. Maximum value" << endl;
    cout << "3. Average value" << endl;
    cout << "Selection: ";
    while(!(cin >> fusion_method)){
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    // Tries to set value and and informs user of the set value
    switch (fusion_method) {
        case 1:
            fuser.setFusionMethod(FUSION_MIN);
            cout << "Selection valid" << endl;
            cout << "Current setting: " << fusion_method << ". Minimum value" << endl;
            break;
        case 2:
            fuser.setFusionMethod(FUSION_MAX);
            cout << "Selection valid" << endl;
            cout << "Current setting: " << fusion_method << ". Maximum value" << endl;
            break;
        case 3:
            fuser.setFusionMethod(FUSION_AVG);
            cout << "Selection valid" << endl;
            cout << "Current setting: " << fusion_method << ". Average value" << endl;
            break;
        default:
            // If the fusion method isn't valid, set it to default of avg
            fusion_method = FUSION_AVG;
            cout << "Selection invalid" << endl;
            cout << "Setting value to default of: 3. Average value" << endl;
            break;
    }



    cout << endl;
    cout << "--------------Initialisation complete---------------" << endl;
    cout << endl;
    cout << "Press ENTER to start generating data, CTRL+C to exit" << endl;
    cin.get();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "------------Commencing data generation--------------" << endl;
    cout << endl;


    //--------------------------------------DATA ACQUISITION---------------------------------------//

//    while(1) {
        // Send container of rangers to the fuser
        fuser.setRangers(rangers2);
    while(1) {
        // Get raw data and print it

        // Print current same number
        cout << endl;
        cout << "------------------Data sample " << fuser.getSampleNum() << ":--------------------" << endl;
        cout << endl;

        std::vector<std::vector<double>> raw_data = fuser.getRawRangeData();
        std::vector<double> temp_data;      // Stores rows of raw data

        cout << "Raw Data: " << endl;
        cout << setprecision(5) << fixed;   // Set data to be printed to 5 decimal places

        for (int i = 0; i<raw_data.size(); i++) {
            temp_data = raw_data.at(i);
            for (int j = 0; j<temp_data.size(); j++) {
                cout << setw(9) << left << temp_data.at(j);
            }
            cout << endl;
        }
        cout << endl;

        // Get fused data and print it
        std::vector<double> fused_data = fuser.getFusedRangeData();

        cout << "Fused Data: " << endl;
        cout << setprecision(5) << fixed;   // Set data to be printed to 5 decimal places

        for (int i = 0; i<fused_data.size(); i++) {
            cout << setw(9) << left << fused_data.at(i);

        }
        cout << endl;
        cout << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}
