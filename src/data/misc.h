/**
 * \headerfile misc.h
 *
 * \brief some functions.
 *
 * \author behley
 */

#ifndef MISC_H_
#define MISC_H_

#include <vector>
#include <string>

/** \brief remove whitespaces at the beginning and end of a string. **/
std::string trim(const std::string& str, const std::string& whitespaces = " \0\t\n\r\x0B");

/** \brief splits a string in tokens, which are seperated by delim. **/
std::vector<std::string> split(const std::string& line, const std::string& delim = " ");

#endif /* MISC_H_ */
