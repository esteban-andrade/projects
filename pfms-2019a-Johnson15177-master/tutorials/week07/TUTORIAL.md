Week 7 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Convars - Ex01
--------------------

notes
-   deadlock is when it will never open

Consider the [starter code](./starter) example which uses two threads to do some work in parallel, questions to reflect on:
* What are the data structures used between threads, what role do the threads have? the purpose is tho do two things, such as generate data and process the data

* Could we use a lock_guard or unique_lock instead of managing the mutex locks 
- where we want to lock we use lock() and unlock(). used to manage mutex
lock_guard() - doesnt give you much control, when you use mutex, you should lock and unlock.

* What are the roles of the sleep_for ? 
- if no sleep_for, it will occupy all processing power of the computer, slowing down the computer
- 

* What is CPU usage of this process? 
- 100%

* What value would you expect to see printed? 
- much less than 100% usage

* If you remove the cout in process thread does that have an effect?


* Do we need two threads?

Compile and run the code

How could we:
* guarantee the process sample runs immediately after new data is generated?
* sort the last N samples generated so the process thread removes the samples closest to the mean?

Questions:
* Where would you keep data?
* Can we embed the data and mutex together?

