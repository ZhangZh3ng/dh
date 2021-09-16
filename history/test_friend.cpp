/*
 * @Author: your name
 * @Date: 2021-08-31 21:13:51
 * @LastEditTime: 2021-09-01 16:43:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/test_friend.cpp
 */

#include <iostream>
#include <string>

class MyHouse
{
public:
    friend class GoodGay;
    friend void visitHouse(const MyHouse &house);
    MyHouse(std::string val_bedroom)
    {
        this->bedroom = val_bedroom;
    }

private:
    std::string bedroom;
};

class GoodGay
{
public:
    GoodGay(MyHouse &val_house):house(val_house){}

    void visit()
    {
        std::cout << this->house.bedroom << std::endl;
    }

private:
    MyHouse &house;
};

void visitHouse(const MyHouse &house)
{
    std::cout << house.bedroom << std::endl;
}

int main()
{
    MyHouse house("dafangzi");
    GoodGay gay = GoodGay(house);
    visitHouse(house);
    gay.visit();
}