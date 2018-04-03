#ifndef Singleton_H
#define Singleton_H

//**********************************************************************************
// Imports
//**********************************************************************************

#include <stddef.h>  // defines NULL

#if defined(_WIN32) || defined (__linux__)
    #include <cassert>
#else
    #define __ASSERT_USE_STDERR
    #include <assert.h>
#endif

//**********************************************************************************
// Singleton <Template> (http://www.yolinux.com/TUTORIALS/C++Singleton.html)
//**********************************************************************************

template <class T>
class Singleton{

    public:
        static T* GetInstance() {
            if(!INSTANCE){
                INSTANCE = new T;
            }
            assert(INSTANCE != NULL);
            return INSTANCE;
        }

    protected:
        Singleton();
        ~Singleton();

    private:
        Singleton(Singleton const&); // Copy Operator
        Singleton& operator=(Singleton const&); // Assignment Operator
        static T* INSTANCE;
};


//========================================================
// Constructor
//========================================================

template <class T>
Singleton<T>::Singleton(){
}

//========================================================
// Instance Definition
//========================================================

template <class T>
T* Singleton<T>::INSTANCE=NULL;

#endif //Singleton_H
