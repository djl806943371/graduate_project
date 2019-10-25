#ifndef SINGLETON_H
#define SINGLETON_H

template<class T>
class Singleton {
private:
    Singleton();
//    Singleton(const T &);
public:
    static T* GetInstance() {
        static T* instance = new T();  //C++11标准下local static对象初始化在多线程条件下安全
        return instance;
    }

private:
    //class members
};

#endif // SINGLETON_H
