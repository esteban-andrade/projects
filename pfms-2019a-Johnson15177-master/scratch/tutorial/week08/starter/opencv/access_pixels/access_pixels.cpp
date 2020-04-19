#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    /*if ( argc != 3 ) {
        printf("usage: ./access_pixels.cpp <Image_Path>\n");
        return -1;
    }*/

    Mat image;
    image = imread("megumin.jpg", CV_LOAD_IMAGE_COLOR);
    Mat modified;
    modified = imread("megumin.jpg", CV_LOAD_IMAGE_COLOR);
    
    if ( !image.data ) {
        printf("No image data \n");
        return -1;
    }
    
    int white_count = 0;
    Vec3b white(255, 255, 255),
    red(0, 0, 255);
   /* for (int i = 0; i<image.rows; i++) {
        for (int j = 0; j <image.cols; j++) {
            
            if (image.at<Vec3b>(i,j)[0] >= 230 && image.at<Vec3b>(i,j)[1] >= 230 && image.at<Vec3b>(i,j)[2] >= 230) {
                white_count++;
                image.at<Vec3b>(i,j)[0] = 0;
                image.at<Vec3b>(i,j)[1] = 0;
                image.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }*/
    
    Point pt1(0, 400);
    Point pt2(400, 400);
    LineIterator line_pt(image, pt1, pt2);
    
    bool did_collide = false;
    for(int i = 0; i < line_pt.count; i++, line_pt++)
    {
        Vec3b *pixel = (Vec3b*)*line_pt;
        if(*pixel == white)
        {
            did_collide = true;
            *pixel = red;
            
        }
        
    }
    
    std::cout << "There are " << white_count << " white-ish pixels" << std::endl;
    namedWindow("Image", WINDOW_AUTOSIZE );
    imshow("Image", image);
    imshow("modified", modified);
    //imwrite(argv[2], image);

    waitKey(0);

    return 0;
}
