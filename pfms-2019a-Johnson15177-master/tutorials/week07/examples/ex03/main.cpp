#include <iostream>
#include <thread>

#include "sample.h"
#include "producer.h"
#include "consumer.h"

using namespace std;


int main ()
{
    Sample sample;
    Producer producer;
    Consumer consumer;

    // Create the threads, the function signature for thread now has two parameters, followed by parameters passed to function
    // - &Class::member_function
    // - object
    // passed parameters to the function
    // - generateSamples takes a sample
    thread inc_thread(&Producer::generateSamples, &producer ,ref(sample));
    thread print_thread(&Consumer::processSamples,&consumer, ref(sample));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}


