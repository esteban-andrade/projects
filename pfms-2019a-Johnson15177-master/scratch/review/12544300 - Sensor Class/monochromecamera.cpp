#include <iostream>
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <vector>

#include "monochromecamera.h"

//set the fixed and default sensor specifications
const int sample_no = 0;
const int sampling_time = 1;
const int min_value = 0;
const int max_value = 255;
const int image_rows = 5;
const int image_columns = 4;

MonochromeCamera::MonochromeCamera() :
    sample_no_(sample_no), sampling_time_(sampling_time), min_value_(min_value), max_value_(max_value),
    image_rows_(image_rows), image_columns_(image_columns) //sets up fixed and default settings
{}

double MonochromeCamera::getSamplingTime()
{
    return sampling_time_;
}

int MonochromeCamera::getMinValue()
{
    return min_value_;
}

int MonochromeCamera::getMaxValue()
{
    return max_value_;
}

int MonochromeCamera::getImageRows()
{
    return image_rows_;
}

int MonochromeCamera::getImageColumns()
{
    return image_columns_;
}

int MonochromeCamera::getSampleNumber()
{
    return sample_no_;
}

std::vector<std::vector<int>> MonochromeCamera::takeImage(char image_size)
{
    image_.clear();
    temp_row_.clear(); //clear both vectors
    if (image_size == 'A') //image size of 5x4
    {
        image_rows_ = 5;
        image_columns_ = 4;
    }
    if (image_size == 'B') //image size of 25x16
    {
        image_rows_ = 25;
        image_columns_ = 16;
    }
    //fill the first row with random values
    for (int x = 0; x < image_columns_; x++)
    {
        seed_ = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed_);
        std::uniform_real_distribution<> value_distribution(min_value_,max_value_ + 1);
        temp_row_.push_back(value_distribution(generator));
    }

    //fill the remaining rows with half the above row
    image_.insert(image_.begin(), temp_row_);
    for (int i = 0; i < (image_rows_ - 1); i++)
    {
        temp_row_.clear();
        for (int j = 0; j < image_columns_; j++)
        {
            temp_row_.push_back(temp_row_[j]/2);
        }
        image_.push_back(temp_row_);
    }
    sample_no_++;
    return image_;
}
