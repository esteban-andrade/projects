#define RECTANGLE_H

class Rectangle { 
public: 
    void setWidthHeight (int,int); 
    int area (void);
    int perimeter(void); 
private:
    int width_, height_; 
};
