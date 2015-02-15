#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>

boost::condition_variable_any cond;
boost::shared_mutex mutex;
std::string str;

using namespace std;

void reader1() {
    boost::shared_lock< boost::shared_mutex > lock(mutex);
    // do work here, without anyone having exclusive access
    while ( str == "" ) {
        cout << "lock the mutex 1 ...\n";
        cond.wait( lock );
    }
    cout << "1 string is " << str << endl;
}

void reader2() {
    boost::shared_lock< boost::shared_mutex > lock(mutex);
    // do work here, without anyone having exclusive access
    while ( str == "" ) {
        cout << "lock the mutex 2 ...\n";
        cond.wait( lock );
    }
    cout << "2 string is " << str << endl;
}

void conditional_writer() {
    boost::upgrade_lock< boost::shared_mutex > lock(mutex);
    // do work here, without anyone having exclusive access
    str = "conditional writer";
    if ( str == "" ) {
        boost::upgrade_to_unique_lock< boost::shared_mutex > unique_lock(lock);
        // do work here, but now you have exclusive access
        str = "upgraded writer";
    }
    cond.notify_all();
    // do more work here, without anyone having exclusive access
}

void unconditional_writer(){
    boost::unique_lock< boost::shared_mutex > lock(mutex);
    cout << "write string\n";
    // do work here, with exclusive access
    str = "unconditional writer";
    cond.notify_all();
}


int main( int argc, char ** argv ) {
    boost::thread t1(reader1);
    boost::thread t2(reader2);
    boost::thread t3(unconditional_writer);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
