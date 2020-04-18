//
// Created by Maciej Czechowski on 18/04/2020.
//

#ifndef PATH_PLANNING_JSON_HELPERS_H
#define PATH_PLANNING_JSON_HELPERS_H

#import <string>
using std::string;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


#endif //PATH_PLANNING_JSON_HELPERS_H
