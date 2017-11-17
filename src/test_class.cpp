#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class A {
public:
    A (){
        std::cout << "A()" << std::endl;
    }
};

class B {
    A a;
public:
    B (){
        std::cout << "B()" << std::endl;
    }
};

class D {
    B b;
public:
    D (){
        std::cout << "D()" << std::endl;
    }
};

class C {
    D d;
public:
    C (){
        std::cout << "C()" << std::endl;
    }
};

int main(int argc, char **argv)
{
    C c;
    return 0;
}


