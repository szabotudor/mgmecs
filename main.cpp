#include "mgmecs.hpp"
#include <ios>
#include <iostream>


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


int main() {
    using namespace mgm;
    std::cout << std::boolalpha;

    MGMecs<> ecs{};

    std::vector<MGMecs<>::Entity> entities{};
    entities.resize(40);
    ecs.create(entities.begin(), entities.end());

    ecs.emplace<int>(entities.begin(), entities.end() - 8, 0);
    ecs.emplace<float>(entities.end() - 12, entities.end(), 16.5f);
    ecs.emplace<double>(entities.end() - 15, entities.end() - 5, 3.1415);

    auto& et = ecs.get_or_emplace<String>(entities[14], "Hello World!");

    const auto& const_ecs = const_cast<const MGMecs<>&>(ecs);

    const auto group = const_ecs.group<double, MGMecs<>::Exclude<int>>();

    for (const auto& e : group) {
        if (e.first == 28) {
            MGMecs<>::Entity* ptr = new MGMecs<>::Entity[5];
            ptr[0] = 25; ptr[1] = 26; ptr[2] = 27; ptr[3] = 31; ptr[4] = 30;
            ecs.destroy(ptr, ptr + 4);
        }
        std::cout << e.first << ' ' << e.second << std::endl;
    }
    std::cout << std::endl;

    return 0;
}
