Week 7 Tutorial Examples
=========================

Examples Ex01-Ex03 introduce convars as a guarantee the process sample runs immediately after new data is generated

Contrast Ex01-Ex03 as they add more modularity with Ex03 having a main that simply connects components and launches threads.

Ex01
--------------------

Main does threading, data, convar and mutex are all declared in main and passed by reference to functions.
The two functions used for threading are in main.cpp and they need to do the thread synchronisation.

Ex02
--------------------

The example creates a class to handle data and syncrohinsation. Thread safe functions are now in class.
Main does threading, data, passing by reference an object to the threaded functions.
The two functions used for threading are in main.cpp.

Ex03
--------------------


The example creates a class to handle data and syncrohinsation. Thread safe functions are now in class.
There is a producer and consumer class with functions internally.
Main does threading, creates objects and interconnects them.

The main is lean, all code is very modular.

Ex04
--------------------

This example is los modular, sorts the last N samples generated so the process thread removes the samples closest to the mean

