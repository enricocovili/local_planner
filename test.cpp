// test yaml loading

#include <iostream>
#include <yaml-cpp/yaml.h>

class Base
{
public:
    Base() {
        std::cout << "Base constructor" << std::endl;
    }
    virtual ~Base() {}
};


class Derived : public Base
{
public:
    Derived() {
        std::cout << "Derived constructor" << std::endl;
    }
    ~Derived() {}
};

int main()
{
    std::shared_ptr<Base> b;
    // std::cout << typeid(*b).name() << std::endl;
    
    b = std::make_shared<Derived>();
    std::cout << typeid(*b).name() << std::endl;
    // get the type of the object
    return 0;
}

// Compile and run the code:
// $ g++ -o test test.cpp -lyaml-cpp