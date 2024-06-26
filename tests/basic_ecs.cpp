#include "mgmecs.hpp"
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <chrono>


struct TestInt {
    TestInt(const TestInt&) = delete;
    TestInt(TestInt&& ti) {
        value = ti.value;
        ti.value = 0;
    }
    TestInt& operator=(const TestInt&) = delete;
    TestInt& operator=(TestInt&& ti) {
        value = ti.value;
        ti.value = 0;
        return *this;
    }
    int value = 0;

    TestInt(const int v) : value{v} {}
    ~TestInt() = default;

    void on_emplace(mgm::MgmEcs& ecs, const mgm::Entity entity) {
        std::cout << "Emplaced " << entity << ' ' << value << std::endl;
    }

    void on_remove(mgm::MgmEcs& ecs, const mgm::Entity entity) {
        std::cout << "Removing " << entity << ' ' << value << std::endl;
    }
};

auto test_mgmecs(const uint32_t num_ents) {
    auto start = std::chrono::high_resolution_clock::now();
    mgm::MgmEcs ecs{};
    std::vector<mgm::Entity> ents{};
    ents.resize(num_ents);
    ecs.create(ents.begin(), ents.end());
    ecs.emplace<int>(ents.begin(), ents.end());
    int i = 467831;
    for (const auto& e : ecs.group<int>()) {
        ecs.get<int>(e) =  i ^= (i << 7) ^ (i >> 13) ^ (i << 8);
    }

    ecs.remove<int>(ents.begin(), ents.end());

    return std::chrono::high_resolution_clock::now() - start;
}


int main() {
    constexpr uint32_t num_iterations = 10000;
    constexpr uint32_t num_entities = 4000;

    {
        for (int i = 0; i < num_iterations / 10; i++) {
            test_mgmecs(num_entities);
        }
        auto start = std::chrono::high_resolution_clock::time_point{};
        auto avg = std::chrono::time_point<std::chrono::high_resolution_clock>{};
        for (int i = 0; i < num_iterations; i++) {
            avg += test_mgmecs(num_entities);
        }
        std::cout << "Average MgMecs Time : " << ((avg - start).count() / num_iterations) << " nanoseconds" << std::endl;
    }

    mgm::MgmEcs ecs{};
    std::cout << "\nTesting callbacks" << std::endl;
    ecs.emplace<TestInt>(ecs.create(), 10);
    ecs.destroy(0);

    std::cout << "\nTesting creating a number of entities and erasing them" << std::endl;
    std::vector<mgm::Entity> ents;
    ents.resize(100);
    ecs.create(ents.begin(), ents.end());
    uint32_t i = 0;
    for (const auto& e : ents) {
        ecs.emplace<uint32_t>(e, i++);
    }

    ents.erase(ents.begin(), ents.begin() + 50);
    ents.erase(ents.begin() + 13, ents.end());
    ecs.destroy(ents.begin(), ents.end());

    std::cout << "\nTesting component references" << std::endl;
    const auto refs = ecs.get_all(1);
    for (const auto& ct : refs) {
        if (ct.type_id == mgm::MgmEcs::type_id<int>)
            std::cout << "This is an int: " << ct.get<int>();
        else if (ct.type_id == mgm::MgmEcs::type_id<uint32_t>)
            std::cout << "This is a uint32_t: " << ct.get<uint32_t>();
        std::cout << std::endl;
    }

    std::cout << "\nTesting looping through a group (should not contain entities 1, 5, 6, 31, 95, 96, 97):\n";
    ecs.emplace<int>(1, 69);
    ecs.emplace<int>(5, 32);
    ecs.emplace<int>(6, 58347);
    ecs.emplace<int>(31, 3);
    ecs.emplace<int>(97);
    ecs.emplace<int>(96);
    ecs.emplace<int>(95);
    for (const auto& e : ecs.group<uint32_t>({}, mgm::TypeList<int>{})) {
        std::cout << e << ' ';
    }
    std::cout << std::endl;

    std::cout << "\nCausing an intentional crash..." << std::endl;
    try {
        std::cout << "Getting an int reference to an int component... " << refs[ecs.type_id<int>].get<int>() << std::endl;
        std::cout << "Getting an int reference to a uint32_t component... " << refs[ecs.type_id<uint32_t>].get<int>() << std::endl;
    } catch (std::runtime_error err) {
        std::cout << "Caught an error: \n\t" << err.what() << std::endl;
    }

    std::cout << "\nAll tests succeded!" << std::endl;
    
    return 0;
}
