#include <iostream>

class too_high_compute_time_exception : public std::exception {
    public:
        const std::string message() {
            return  "Maximum computation time exceeded. ";
        }
};