#include "cgrlib.hpp"
#include <climits>
#include <vector>
#include <map>
#include <algorithm>
#include <exception>
#include <cassert>
#include <fstream>
#include <ostream>
#include <string>
#include <iostream>
#include <stdlib.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
// #include "../../../common/util/include/JsonSerializable.h"
// #include "JsonSerializable.h"


#define MAX_SIZE INT_MAX
// #define MAX_SIZE 100000
// or UINT_MAX
#define DEBUG

template <typename T>
bool vector_contains(std::vector<T> vec, T ele) {
    auto it = std::find(vec.begin(), vec.end(), ele);
    return it != std::end(vec);
}


// Throw this exception in methods that would otherwise try to access an empty container
class EmptyContainerError: public std::exception {
    virtual const char* what() const throw() {
        return "Tried to access element of an empty container";
    }
};

void Contact::clear_dijkstra_working_area() {
    arrival_time = MAX_SIZE;
    visited = false;
    predecessor = NULL;
    visited_nodes.clear();
}

bool Contact::operator==(const Contact contact) const {
    return (frm == contact.frm &&
            to == contact.to &&
            start == contact.start &&
            end == contact.end &&
            rate == contact.rate &&
            owlt == contact.owlt &&
            confidence == contact.confidence);
}

bool Contact::operator!=(const Contact contact) const {
    return !(*this == contact);
}

std::ostream& operator<<(std::ostream &out, const Contact &obj) {
    // return "%s->%s(%s-%s,d%s)[mav%d%%]" % (self.frm, self.to, self.start, end, self.owlt, volume)
    static const boost::format fmtTemplate("%d->%d(%d-%d,d%d[mav%f%%]");
    boost::format fmt(fmtTemplate);

    int min_vol = *std::min_element(obj.mav.begin(), obj.mav.end());
    // int min_vol = 100;
    float volume = 100 * min_vol / obj.volume;
    // int volume = obj.volume;
    fmt % obj.frm % obj.to % obj.start % obj.end % obj.owlt % volume;
    const std::string message(std::move(fmt.str()));

    out << message;
    return out;
}

Contact::Contact(int frm, int to, int start, int end, int rate, float confidence, int owlt)
    : frm(frm), to(to), start(start), end(end), rate(rate), confidence(confidence), owlt(owlt)
{
    // fixed parameters
    volume = rate * (end - start);

    // variable parameters
    mav = std::vector<int>({volume, volume, volume});

    // route search working area
    arrival_time = MAX_SIZE;
    visited = false;
    visited_nodes = std::vector<int>();
    predecessor = NULL;

    // route management working area
    suppressed = false;
    suppressed_next_hop = std::vector<Contact>();

    // forwarding working area
    // first_byte_tx_time = NULL;
    // last_byte_tx_time = NULL;
    // last_byte_arr_time = NULL;
    // effective_volume_limit = NULL;
}
    
Contact::Contact() {}
Contact::~Contact() {}

std::ostream& operator<<(std::ostream &out, const Route &obj) {
    "to:%s|via:%s(%03d,%03d)|bdt:%s|hops:%s|vol:%s|conf:%s|%s";
    // static const boost::format fmtTemplate("to:%d|via:%d(%03d,%03d)|bdt:%d|hops:%d|vol:%d|conf:%f|%s");
    static const boost::format fmtTemplate("to:%d|via:%d(%03d,%03d)|bdt:%d|vol:%d|conf:%f");
    boost::format fmt(fmtTemplate);

    // const std::vector<Contact> routeHops = obj.get_hops();

    // fmt % obj.to_node % obj.next_node % obj.from_time % obj.to_time % obj.best_delivery_time
    //     % routeHops.size() % obj.volume % obj.confidence % routeHops;
    fmt % obj.to_node % obj.next_node % obj.from_time % obj.to_time % obj.best_delivery_time
        % obj.volume % obj.confidence;
    const std::string message(std::move(fmt.str()));

    out << message;
    return out;
}

Route::Route() {}
Route::~Route() {}

Route::Route(Contact contact, Route *parent)
    : parent(parent)
{
    hops = std::vector<Contact>();
    if (NULL == parent) {
        // to_node = NULL;
        // next_node = NULL;
        from_time = 0;
        to_time = MAX_SIZE;
        best_delivery_time = 0;
        volume = MAX_SIZE;
        confidence = 1;
        __visited = std::map<int, bool>();
    } else {
        to_node = parent->to_node;
        next_node = parent->next_node;
        from_time = parent->from_time;
        to_time = parent->to_time;
        best_delivery_time = parent->best_delivery_time;
        volume = parent->volume;
        confidence = parent->confidence;
        __visited = std::map<int, bool>(parent->__visited); // copy
    }

    append(contact);
}

Contact Route::get_last_contact() {
    if (hops.empty()) {
        throw EmptyContainerError();
    } else {
        return hops.back();
    }
}

bool Route::visited(int node) {
    return __visited.count(node) && __visited[node];
}

void Route::append(Contact contact) {
    assert(eligible(contact));
    hops.push_back(contact);
    __visited[contact.frm] = true;
    __visited[contact.to] = true;

    refresh_metrics();
}

void Route::refresh_metrics() {
    assert(!hops.empty());
    to_node = get_hops().back().to;
    next_node = get_hops()[0].to;
    from_time = get_hops()[0].start;
    to_time = MAX_SIZE;
    best_delivery_time = 0;
    confidence = 1;
    for (Contact contact : get_hops()) {
        to_time = std::min(to_time, contact.end);
        best_delivery_time = std::max(best_delivery_time + contact.owlt, contact.start + contact.owlt);
        confidence *= contact.confidence;
    }

    // volume
    int prev_last_byte_arr_time = 0;
    int min_effective_volume_limit = MAX_SIZE;
    for (Contact contact : get_hops()) {
        if (contact == get_hops()[0]) {
            contact.first_byte_tx_time = contact.start;
        } else {
            contact.first_byte_tx_time = std::max(contact.start, prev_last_byte_arr_time);
        }
        int bundle_tx_time = 0;
        contact.last_byte_tx_time = contact.first_byte_tx_time + bundle_tx_time;
        contact.last_byte_arr_time = contact.last_byte_tx_time + contact.owlt;
        prev_last_byte_arr_time = contact.last_byte_arr_time;
        
        int effective_start_time = contact.first_byte_tx_time;
        int min_succ_stop_time = MAX_SIZE;
        auto it = std::find(get_hops().begin(), get_hops().end(), contact);
        for (it; it < get_hops().end(); ++it) {
            Contact successor = *it;
            if (successor.end < min_succ_stop_time) {
                min_succ_stop_time = successor.end;
            }
        }
        int effective_stop_time = std::min(contact.end, min_succ_stop_time);
        int effective_duration = effective_stop_time - effective_start_time;
        contact.effective_volume_limit = std::min(effective_duration * contact.rate, contact.volume);
        if (contact.effective_volume_limit < min_effective_volume_limit) {
            min_effective_volume_limit = contact.effective_volume_limit;
        }
    }
    volume = min_effective_volume_limit;
}

bool Route::eligible(Contact contact) {
    try {
        Contact last = get_last_contact();
        return (!visited(contact.to) && contact.end > last.start + last.owlt);
    } catch (EmptyContainerError) {
        return true;
    }
}

std::vector<Contact> Route::get_hops() {
    if (NULL == parent) {
        return hops;
    } else {
        std::vector<Contact> v(parent->get_hops());
        v.insert(v.end(), hops.begin(), hops.end());
        return v;
    }
}

std::vector<Contact> cp_load(std::string filename, int max_contacts=MAX_SIZE) {
    std::vector<Contact> contactsVector;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(filename, pt);
    const boost::property_tree::ptree & contactsPt
        = pt.get_child("contacts", boost::property_tree::ptree());
    BOOST_FOREACH(const boost::property_tree::ptree::value_type & eventPt, contactsPt) {
        Contact new_contact = Contact(eventPt.second.get<int>("source", 0),
                                      eventPt.second.get<int>("dest", 0),
                                      eventPt.second.get<int>("startTime", 0),
                                      eventPt.second.get<int>("endTime", 0),
                                      eventPt.second.get<int>("rate", 0),
                                      1.0, // confidence
                                      1);
        // eventPt.second.get<int>("owlt", 1));
        contactsVector.push_back(new_contact);
        if (contactsVector.size() == max_contacts) {
            break;
        }
    }
    return contactsVector;
}

Route dijkstra(Contact *root_contact, int destination, std::vector<Contact> contact_plan) {
    for (Contact& contact : contact_plan) {
        if (contact != *root_contact) {
            contact.clear_dijkstra_working_area();
        }
    }

    // Make sure we map to pointers so we can modify the underlying contact_plan
    // using the hash.
    std::map<int, std::vector<Contact*>> contact_plan_hash;
    for (std::vector<Contact>::iterator contact = contact_plan.begin(); contact != contact_plan.end(); ++contact) {
        if (!contact_plan_hash.count(contact->frm)) {
            contact_plan_hash[contact->frm] = std::vector<Contact*>();
        }
        if (!contact_plan_hash.count(contact->to)) {
            contact_plan_hash[contact->to] = std::vector<Contact*>();
        }
        contact_plan_hash[contact->frm].push_back(&(*contact));
    }

    Route route;
    Contact *final_contact = NULL;
    int earliest_fin_arr_t = MAX_SIZE;
    int arrvl_time;

    Contact *current = root_contact;

    if (!vector_contains(root_contact->visited_nodes, root_contact->to)) {
        root_contact->visited_nodes.push_back(root_contact->to);
    }

    std::cout << "Dijkstra from " << *root_contact << " to " << destination << " arrival time: " << root_contact->arrival_time << std::endl;
    while (true) {
        std::cout << "Current contact: " << *current << std::endl;
        for (Contact* contact : contact_plan_hash[current->to]) {
            std::cout << "Explore contact: " << *contact << " - ";
            if (vector_contains(current->suppressed_next_hop, *contact)) {
                std::cout << "\tignore (suppressed_next_hop - Yens')" << std::endl;
                continue;
            }
            if (contact->suppressed) {
                std::cout << "\tignore (suppressed)" << std::endl;
                continue;
            }
            if (contact->visited) {
                std::cout << "\tignore (contact visited)" << std::endl;
                continue;
            }
            if (vector_contains(current->visited_nodes, contact->to)) {
                std::cout << "\tignore (node visited)" << std::endl;
                continue;
            }
            if (contact->end <= current->arrival_time) {
                std::cout << "\tignore (contact ends before arrival_time) " << contact->end << ' ' << current->arrival_time << std::endl;
                continue;
            }
            if (*std::max_element(contact->mav.begin(), contact->mav.end()) <= 0) {
                std::cout << "\tignore (no residual volume)" << std::endl;
                continue;
            }
            if (current->frm == contact->to && current->to == contact->frm) {
                std::cout << "\tignore (return to previous node)" << std::endl;
                continue;
            }
            std::cout << "\tcontact not ignored - ";

            // Calculate arrival time (cost)
            if (contact->start < current->arrival_time) {
                arrvl_time = current->arrival_time + contact->owlt;
                std::cout << "arrival_time: " << arrvl_time << " - ";
            } else {
                arrvl_time = contact->start + contact->owlt;
                std::cout << "arrival_time: " << arrvl_time << " - ";
            }

            if (arrvl_time <= contact->arrival_time) {
                std::cout << "updated from: " << contact->arrival_time << " - ";
                contact->arrival_time = arrvl_time;
                contact->predecessor = current;
                contact->visited_nodes = current->visited_nodes;
                contact->visited_nodes.push_back(contact->to);
                
                if (contact->to == destination && contact->arrival_time < earliest_fin_arr_t) {
                    std::cout << "marked as final! - ";
                    earliest_fin_arr_t = contact->arrival_time;
                    final_contact = &(*contact);
                }
            } else {
                std::cout << "not updated (previous: " << contact->arrival_time << ") - ";
            }
            std::cout << "done" << std::endl;
        }

        current->visited = true;

        // Determine next best contact
        int earliest_arr_t = MAX_SIZE;
        Contact *next_contact = NULL;

        for (Contact& contact : contact_plan) {
            if (contact.suppressed || contact.visited) {
                continue;
            }
            if (contact.arrival_time > earliest_fin_arr_t) {
                continue;
            }
            if (contact.arrival_time < earliest_arr_t) {
                earliest_arr_t = contact.arrival_time;
                next_contact = &contact;
            }
        }

        if (NULL == next_contact) {
            break;
        }

        current = next_contact;
    }

    if (final_contact != NULL) {
        std::vector<Contact> hops;
        Contact contact = *final_contact;
        while (contact != *root_contact) {
            hops.push_back(contact);
            contact = *contact.predecessor;
        }
        
        route = Route(hops.back());
        // exit(1);
        hops.pop_back();
        while (!hops.empty()) {
            route.append(hops.back());
            hops.pop_back();
        }
    }

    return route;
}

int main() {

    std::vector<Contact> contact_plan = cp_load("cgrTutorial.json", MAX_SIZE);

    std::cout << "---contact plan---" << std::endl;
    std::cout << "[";
    for (int i = 0; i < contact_plan.size()-1; i++) {
        std::cout << contact_plan[i] << ", ";
    }
    std::cout << contact_plan[contact_plan.size()-1] << "]" << std::endl;

    int source = 1;
    int destination = 5;
    int curr_time = 0;

// dijkstra returns a single (best) route from contact plan
    std::cout << "---dijkstra---" << std::endl;
    Contact root = Contact(source, source, 0, MAX_SIZE, 100, 1.0, 0);
    root.arrival_time = curr_time;

    Route result = dijkstra(&root, destination, contact_plan);

    // std::cout << result.next_node << std::endl;
    std::cout << result << std::endl;

    return 0;
}
