#ifndef LIB_CGR_H
#define LIB_CGR_H

#include <vector>
#include <map>
#include <ostream>
#include <limits>

namespace cgr {

const int MAX_SIZE = std::numeric_limits<int>::max();

class Contact {
public:
    // Fixed parameters
    int frm, to, start, end, rate, volume;
    int owlt;
    // int id = -1;
    float confidence;
    // Variable parameters
    std::vector<int> mav;
    // Route search working area
    int arrival_time;
    bool visited;
    Contact *predecessor;
    std::vector<int> visited_nodes;
    // Route management working area
    bool suppressed;
    std::vector<Contact> suppressed_next_hop;
    // Forwarding working area
    int first_byte_tx_time, last_byte_tx_time, last_byte_arr_time, effective_volume_limit;
    void clear_dijkstra_working_area();
    Contact(int frm, int to, int start, int end, int rate, float confidence=1, int owlt=1);
    Contact();
    ~Contact();
    bool operator==(const Contact) const;
    bool operator!=(const Contact) const;
    friend std::ostream& operator<<(std::ostream&, const Contact&);
};


class Route {
public:
    int to_node, next_node, from_time, to_time, best_delivery_time, volume;
    float confidence;
    Route(Contact, Route *parent=NULL);
    Route();
    ~Route();
private:
    Route *parent;
    std::vector<Contact> hops;
    std::map<int, bool> __visited;
public:
    Contact get_last_contact();
    bool visited(int node);
    void append(Contact contact);
    void refresh_metrics();
    bool eligible(Contact contact);
    std::vector<Contact> get_hops();
    friend std::ostream& operator<<(std::ostream&, const Route&);
};

std::vector<Contact> cp_load(std::string filename, int max_contacts=MAX_SIZE);

Route dijkstra(Contact *root_contact, int destination, std::vector<Contact> contact_plan);

template <typename T>
bool vector_contains(std::vector<T> vec, T ele);

class EmptyContainerError: public std::exception {
    virtual const char* what() const throw();
};

std::ostream& operator<<(std::ostream &out, const std::vector<Contact> &obj);
std::ostream& operator<<(std::ostream &out, const Contact &obj);
std::ostream& operator<<(std::ostream &out, const Route &obj);

} // namespace cgr

#endif // LIB_CGR_H
