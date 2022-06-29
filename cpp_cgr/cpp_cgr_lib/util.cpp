#include "cgrlib.hpp"
#include <vector>
#include <algorithm>
#include <exception>
#include <ostream>

#include <boost/format.hpp>

template <typename T>
bool vector_contains(std::vector<T> vec, T ele) {
    auto it = std::find(vec.begin(), vec.end(), ele);
    return it != std::end(vec);
}

// Throw this exception in methods that would otherwise try to access an empty container
// E.g. calling std::vector::back() on an empty vector is undefined behavior.
// Following the approach of the Python version CGR library, we use this class for Route::eligible()
// as a substitue for Python's IndexError
class EmptyContainerError: public std::exception {
    virtual const char* what() const throw() {
        return "Tried to access element of an empty container";
    }
};

std::ostream& operator<<(std::ostream &out, const std::vector<Contact> &obj) {
    out << "[";
    for (int i = 0; i < obj.size()-1; i++) {
        out << obj[i] << ", ";
    }
    out << obj[obj.size()-1] << "]";

    return out;
}

std::ostream& operator<<(std::ostream &out, const Contact &obj) {
    static const boost::format fmtTemplate("%d->%d(%d-%d,d%d)[mav%.0f%%]");
    boost::format fmt(fmtTemplate);

    int min_vol = *std::min_element(obj.mav.begin(), obj.mav.end());
    float volume = 100 * min_vol / obj.volume;
    fmt % obj.frm % obj.to % obj.start % obj.end % obj.owlt % volume;
    const std::string message(std::move(fmt.str()));

    out << message;
    return out;
}

std::ostream& operator<<(std::ostream &out, const Route &obj) {
    static const boost::format fmtTemplate("to:%d|via:%d(%03d,%03d)|bdt:%d|hops:%d|vol:%d|conf:%.1f|%s");
    boost::format fmt(fmtTemplate);

    std::vector<Contact> routeHops = static_cast<Route>(obj).get_hops();

    fmt % obj.to_node % obj.next_node % obj.from_time % obj.to_time % obj.best_delivery_time
        % routeHops.size() % obj.volume % obj.confidence % routeHops;
    const std::string message(std::move(fmt.str()));

    out << message;
    return out;
}
