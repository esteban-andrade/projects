#include <iostream>
#include <string.h>
#include <vector>
#include <chrono>

#include "rangerfusioninterface.h"
#include "rangerfusion.h"
#include "rangerinterface.h"
#include "ranger.h"
#include "laser.h"
#include "radar.h"
#include "generator.h" 

using namespace std;

int main () 
{
    int input;
    /* create sensor objects and store in STL of sensors*/
    Laser laser1;
    Radar radar1;
    Radar radar2;
    
    vector<RangerInterface*> rangers(3);

    rangers.at(0) = (&laser1);
    rangers.at(1) = (&radar1);
    rangers.at(2) = (&radar2);

    RangerFusion fusion;
    fusion.setRangers(rangers);

    /* ----------Fixed Laser perameters---------- */
    cout << "-----Laser Fixed Parameters-----" << endl;
    cout << "Model: " << laser1.getModel() << endl;
    cout << "Field of View: " << laser1.getFieldOfView() << " degrees" << endl;
    cout << "Scanning Distances: " << laser1.getMinRange() << "m to " << laser1.getMaxRange() << "m" << endl;
    cout << "Orientation Offset: " << laser1.getOffset() << endl;
    cout << endl;
    
    /* ----------Setting laser parameters---------- */
    cout << "-----Setting non-fixed parameters for laser1-----" << endl;
    
    /* Set Angular Resolution */
    cout << "Please Select an Angular Resolution (10 or 30 degrees): ";
    cin >> input;
    laser1.setAngularResolution(input);
    if (laser1.setAngularResolution(input) == false)
    {
        cout << "You have not selected an invalid input, using default Angular Resolution" << endl;
    }
    cout << "Angular Resolution is set to " << laser1.getAngularResolution() << " degrees" << endl;
    cout << endl;
    
    /* ----------Fixed Radar parameters---------- */
    cout << "-----Radar Fixed Parameters-----" << endl;
    cout << "Model: " << radar1.getModel() << endl;
    cout << "Field of View: " << radar1.getFieldOfView() << " degrees" << endl;
    cout << "Angular Resolution: " << radar1.getAngularResolution() << " degrees" << endl;
    cout << "Scanning distances: " << radar1.getMinRange() << "m to " << radar1.getMaxRange() << "m" << endl;
    cout << endl;
    
    /* ----------Setting Radar1 parameters---------- */
    cout << "-----Setting non-fixed parameters for radar1-----" << endl;
    cout << "Select an offset between 120 and -120 degrees (must be multiple of 10): ";
    cin >> input;
    radar1.setOffset(input);
    if (radar1.setOffset(input) == false)
    {
        cout << "You have not selected a valid input, using default Offset" << endl;
    }
    cout << "Offset is set to " << radar1.getOffset() << " degrees" <<endl;
    cout << endl;

    /* ----------Setting Radar2 parameters---------- */
    cout << "-----Setting non-fixed parameters for radar2-----" << endl;
    cout << "Select an offset between 120 and -120 degrees (must be multiple of 10): ";
    cin >> input;
    radar2.setOffset(input);
    if (radar2.setOffset(input) == false)
    {
        cout << "You have not selected a valid input, using default Offset" << endl;
    }
    cout << "Offset is set to " << radar2.getOffset() << " degrees" <<endl;
    cout << endl;


     /* ----- All Laser1 Parameters Printed ----- */
    cout << "----- Laser1 Parameters Set -----" << endl;
    cout << "Model: " << laser1.getModel() << endl;
    cout << "FOV: " << laser1.getFieldOfView() << " degrees" << endl;
    cout << "Offset: " << laser1.getOffset() << " degrees" << endl;
    cout << "AngRes: " << laser1.getAngularResolution() << " degrees" << endl;
    cout << "Min Distance: " << laser1.getMinRange() << "m" << endl;
    cout << "Max Distance: " << laser1.getMaxRange() << "m" << endl;
    cout << endl;
    
    
    /* ----- All Radar1 Parameters Printed ----- */
    cout << "----- Radar1 Parameters Set -----" << endl;
    cout << "Model: " << radar1.getModel() << endl;
    cout << "FOV: " << radar1.getFieldOfView() << " degrees" << endl;
    cout << "Offset: " << radar1.getOffset() << " degrees" << endl;
    cout << "AngRes: " << radar1.getAngularResolution() << " degrees" << endl;
    cout << "Min Distance: " << radar1.getMinRange() << "m" << endl;
    cout << "Max Distance: " << radar1.getMaxRange() << "m" << endl;
    cout << endl;
    
    /* ----- All Radar2 Parameters Printed ----- */
    cout << "----- Radar2 Parameters Set -----" << endl;
    cout << "Model: " << radar2.getModel() << endl;
    cout << "FOV: " << radar2.getFieldOfView() << " degrees" << endl;
    cout << "Offset: " << radar2.getOffset() << " degrees" << endl;
    cout << "AngRes: " << radar2.getAngularResolution() << " degrees" << endl;
    cout << "Min Distance: " << radar2.getMinRange() << "m" << endl;
    cout << "Max Distance: " << radar2.getMaxRange() << "m" << endl;
    cout << endl;

    cout << "Please choose a fusion method (0=min, 1=max, 2=avg)" <<endl;

    cin >> input;

        if (input == FUSION_MIN)
        {
            cout << "Min Method Selected" << endl;
            fusion.takeInput(input);
            while(1)
            {
                vector<vector<double>> RawData = fusion.getRawRangeData();
                cout << "------------------------------------------------------------------------------------" << endl;
                cout << "The laser data" << endl;
                for(int a=0; a<RawData.at(0).size(); a++)
                {
                    cout << RawData.at(0).at(a) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar1 data" << endl;
                for(int b=0; b<RawData.at(1).size(); b++)
                {
                    cout << RawData.at(1).at(b) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar2 data" << endl;
                for(int b=0; b<RawData.at(2).size(); b++)
                {
                    cout << RawData.at(2).at(b) << " ";
                }
                cout << endl;
                cout << endl;
                fusion.setFusionMethod(FUSION_MIN);

                vector<double> FusedData = fusion.getFusedRangeData();

                cout << "The fused data" << endl;
                for(int i=0; i<FusedData.size(); i++)
                {
                    cout << FusedData.at(i) << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------" << endl;
            }
        }

        else if (input == FUSION_MAX)
        {
            cout << "Max Method Selected" << endl;
            fusion.takeInput(input);
            while(1)
            {
                vector<vector<double>> RawData = fusion.getRawRangeData();
                cout << "------------------------------------------------------------------------------------" << endl;
                cout << "The laser data" << endl;
                for(int a=0; a<RawData.at(0).size(); a++)
                {
                    cout << RawData.at(0).at(a) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar1 data" << endl;
                for(int b=0; b<RawData.at(1).size(); b++)
                {
                    cout << RawData.at(1).at(b) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar2 data" << endl;                cout << endl;
                for(int b=0; b<RawData.at(2).size(); b++)
                {
                    cout << RawData.at(2).at(b) << " ";
                }
                cout << endl;
                cout << endl;
                fusion.setFusionMethod(FUSION_MAX);

                vector<double> FusedData = fusion.getFusedRangeData();

                cout << "The fused data" << endl;
                for(int i=0; i<FusedData.size(); i++)
                {
                    cout << FusedData.at(i) << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------" << endl;
            }
        }

        else if (input == FUSION_AVG)
        {
            cout << "Avg Method Selected" << endl;
            fusion.takeInput(input);
            while(1)
            {
                vector<vector<double>> RawData = fusion.getRawRangeData();
                cout << "------------------------------------------------------------------------------------" << endl;
                cout << "The laser data" << endl;
                for(int a=0; a<RawData.at(0).size(); a++)
                {
                    cout << RawData.at(0).at(a) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar1 data" << endl;
                for(int b=0; b<RawData.at(1).size(); b++)
                {
                    cout << RawData.at(1).at(b) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar2 data" << endl;
                for(int b=0; b<RawData.at(2).size(); b++)
                {
                    cout << RawData.at(2).at(b) << " ";
                }
                cout << endl;
                cout << endl;
                fusion.setFusionMethod(FUSION_AVG);

                vector<double> FusedData = fusion.getFusedRangeData();

                cout << "The fused data" << endl;
                for(int i=0; i<FusedData.size(); i++)
                {
                    cout << FusedData.at(i) << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------" << endl;
            }
        }
    
        else
        {
            cout << "invalid choice, using default" << endl;
            cout << "Min Method Selected" << endl;
            fusion.takeInput(input);
            while(1)
            {
                vector<vector<double>> RawData = fusion.getRawRangeData();
                cout << "------------------------------------------------------------------------------------" << endl;
                cout << "The laser data" << endl;
                for(int a=0; a<RawData.at(0).size(); a++)
                {
                    cout << RawData.at(0).at(a) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar1 data" << endl;
                for(int b=0; b<RawData.at(1).size(); b++)
                {
                    cout << RawData.at(1).at(b) << " ";
                }
                cout << endl;
                cout << endl;

                cout << "The radar2 data" << endl;
                for(int b=0; b<RawData.at(2).size(); b++)
                {
                    cout << RawData.at(2).at(b) << " ";
                }
                cout << endl;
                cout << endl;
                fusion.setFusionMethod(FUSION_MIN);

                vector<double> FusedData = fusion.getFusedRangeData();

                cout << "The fused data" << endl;
                for(int i=0; i<FusedData.size(); i++)
                {
                    cout << FusedData.at(i) << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------" << endl;
            }
        }
    
    return 0;
} 
