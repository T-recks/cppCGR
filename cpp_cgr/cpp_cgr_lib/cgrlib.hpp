#include <vector>
#include <map>
#include <cstddef>
#include <ostream>

class Contact {
public:
    // Fixed parameters
    int frm, to, start, end, rate, volume;
    int owlt;
    // int id;
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
    Contact(int frm, int to, int start, int end, int rate, float confidence=1, int owlt=0);
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
