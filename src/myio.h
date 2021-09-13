/*
 * @Author: your name
 * @Date: 2021-09-13 22:07:52
 * @LastEditTime: 2021-09-13 22:09:41
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/myio.h
 */
#ifndef DH_MYIO_H
#define DH_MYIO_H

#include <iostream>

namespace dh{

  template<typename T>
  std::ostream& operator<<(std::ostream&os, const std::vector<T>& data){
    for (typename std::vector<T>::const_iterator it = data.begin();
         it != data.end();
         ++it)
    {
      os << *it << std::endl;
    }
    return os;
  }
  
}
#endif

