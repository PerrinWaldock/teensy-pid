#include "singleton.h"

Singleton* Singleton::singleton_= nullptr;

Singleton::Singleton()
{
}

Singleton *Singleton::GetInstance()
{
    if (singleton_ == nullptr){
        singleton_ = new Singleton();
    }
    return singleton_;
}