#include "libcgr.cpp"
#include <vector>
#include <iostream>

using namespace cgr;

/*
 * A simple tutorial/example program
 */
int main() {
    std::vector<Contact> contact_plan = cp_load("cgrTutorial.json", MAX_SIZE);

    std::cout << "---contact plan---" << std::endl;
    std::cout << contact_plan << std::endl;

    int source = 1;
    int destination = 5;
    int curr_time = 0;

    // dijkstra returns a single (best) route from contact plan
    std::cout << "---dijkstra---" << std::endl;
    Contact root = Contact(source, source, 0, MAX_SIZE, 100, 1.0, 0);
    root.arrival_time = curr_time;

    Route best = dijkstra(&root, destination, contact_plan);

    std::cout << best << std::endl;
    // std::cout << best.next_node << std::endl;

    return 0;
}
