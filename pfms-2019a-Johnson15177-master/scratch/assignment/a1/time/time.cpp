#include <iostream>
#include <chrono>
#include <thread>

static bool s_Finished = false;
int i=0;

void DoWork()
{
    //using namespace std::literals::chrono_literals;    
    while(i<10)
    {
    std::cout << "Working...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    i++;
    }
    
   
    s_Finished = true;
}
 
int main()
{
    int i;
  //  std::thread worker(DoWork);
  //  DoWork();
    std::cin >> i;
    switch (i) {
        case 1: std::cout << "pressed 1\n";
                break;
        case 2: std::cout << "pressed 2\n";
                break;
        case 3: std::cout << "pressed 3\n";
                break;
        case 4: std::cout << "pressed 4\n";
                break;
        default: std::cout << "error\n";
    }
   // std::cin.get( );

    //s_Finished = true;
    
    //worker.join();
    
  //  std::cin.get( );
}
