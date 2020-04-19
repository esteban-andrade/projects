#include <iostream>
#include <limits>
#include <vector>
#include <unistd.h>

#include "monochromecamera.h"



int main()
{
    //create object of camera
    std::cout << "Initialising sensor...\n";
    MonochromeCamera ccd_mno;
    std::cout << "Initialising Complete...\n\n";

    //query sensor fixed parameters
    std::cout << "Model: CCD-MN0\nSampling Rate: " << ccd_mno.getSamplingTime() << "Hz\nMin Value: "
              << ccd_mno.getMinValue() << "\nMax Value: " << ccd_mno.getMaxValue() << '\n';


    //ask user to specify image size. 5x4 or 25x16
    std::cout << "Choose an image size:\n[A] 5x4\n[B] 25x16\n";
    char image_size;
    std::cin >> image_size;
    if (image_size == 'A')
    {
        std::cout << "You have selected an image size of: 5x4\n\n";
    }
    else if (image_size == 'B')
    {
        std::cout << "You have selected an image size of: 25x16\n\n";
    }
    else
    {
        image_size = 'A';
        std::cout << "INVALID INPUT! Default image size of 5x4 used\n\n";
    }

    //continuously queries and dislays data from camera
    while(1)
    {
        std::vector<std::vector<int>> image = ccd_mno.takeImage(image_size);
        std::cout << "Sample Number: [" << ccd_mno.getSampleNumber() << "]\n";

        //prints the image
        for (int i = 0; i < ccd_mno.getImageRows(); i++)
        {
            for (int j = 0; j < ccd_mno.getImageColumns(); j++)
            {
                std::cout.width(4);
                std::cout << image[i][j];
            }
            std::cout << '\n';
        }
        std::cout << "\n";
        usleep(1*1000000/(ccd_mno.getSamplingTime())); //sleeps program for 1 second
    }
    return 0;
}
