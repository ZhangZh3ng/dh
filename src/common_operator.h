/*
 * @Author: your name
 * @Date: 2021-09-13 22:07:52
 * @LastEditTime: 2021-09-17 14:45:07
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/common_operator.h
 */
#ifndef DH_COMMON_OPERATOR_H
#define DH_COMMON_OPERATOR_H

#include <iostream>
#include <vector>

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

  template <typename T>
  std::vector<T> operator-(const std::vector<T> &a,
                           const std::vector<T> &b){
    std::vector<T> diff;
    diff.clear();
    if(a.size() != b.size())
      return diff;

    for (typename std::vector<T>::const_iterator ita = a.begin(), itb = b.begin();
         ita != a.end();
         ++ita, ++itb)
    {
      diff.push_back((*ita) - (*itb));
    }
    return diff;
  }

} // namespace dh
#endif // DH_MYIO_H

