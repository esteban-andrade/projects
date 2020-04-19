#ifndef MONOCHROMECAMERA_H
#define MONOCHROMECAMERA_H

#include <iostream>
#include <vector>

class MonochromeCamera
{
public:
    MonochromeCamera();
    double getSamplingTime();
    int getMinValue();
    int getMaxValue();
    int getImageRows();
    int getImageColumns();
    int getSampleNumber();
    std::vector<std::vector<int>> takeImage(char image_size);
private:
    int sampling_time_; //in Hz
    unsigned int min_value_;
    unsigned int max_value_;
    int image_rows_;
    int image_columns_;
    std::vector<std::vector<int>> image_;
    std::vector<int> temp_row_;
    int sample_no_;
    unsigned int seed_;
};

#endif // MONOCHROMECAMERA_H
