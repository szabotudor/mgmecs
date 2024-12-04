#include "mgmecs.hpp"
#include <ios>
#include <iostream>
#include <chrono>


struct String {
    std::string str{};

    String(const std::string& s) : str{s} {}

    void on_construct(mgm::MGMecs<>* ecs, mgm::MGMecs<>::Entity e) {
        std::cout << "Constructed \"" << str << '"' << std::endl;
    }

    void on_destroy(mgm::MGMecs<>* ecs, const mgm::MGMecs<>::Entity e) {
        std::cout << "Destroyed \"" << str << '"' << std::endl;
    }
};


auto test_mgmecs() {
    using namespace mgm;

    std::chrono::high_resolution_clock::duration avg_time{};

    for (size_t i = 0; i < 10000; i++) {
        const auto start = std::chrono::high_resolution_clock::now();

        MGMecs<> ecs{};

        std::vector<MGMecs<>::Entity> entities{};
        entities.resize(40);
        ecs.create(entities.begin(), entities.end());

        ecs.emplace<int>(entities.begin(), entities.end() - 8, 0);
        ecs.emplace<float>(entities.end() - 12, entities.end(), 16.5f);
        ecs.emplace<double>(entities.end() - 15, entities.end() - 5, 3.1415);

        auto& et = ecs.get_or_emplace<String>(entities[14], "Hello World!");

        for (const auto& e : ecs.group<double>()) {
            if (e.first == 28) {
                MGMecs<>::Entity* ptr = new MGMecs<>::Entity[5];
                ptr[0] = 25; ptr[1] = 26; ptr[2] = 27; ptr[3] = 31; ptr[4] = 30;
                ecs.destroy(ptr, ptr + 2);
            }
            std::cout << e.first << ' ' << ecs.get<double>(e.first) << std::endl;
        }
        std::cout << std::endl;
        
        const auto end = std::chrono::high_resolution_clock::now();

        avg_time += end - start;
    }
    avg_time /= 10000;
    
    return avg_time;
}


int main() {
    using namespace mgm;
    std::cout << std::boolalpha;

    std::cout << "Starting tests...\n";

    const auto _mgmecs = test_mgmecs().count();

    std::cout << "MGMecs: " << _mgmecs << "ns\n";

    return 0;
}
