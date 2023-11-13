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
};

auto test_mgmecs(const uint32_t num_ents) {
    auto start = std::chrono::high_resolution_clock::now();
    mgm::MgmEcs ecs{};
    auto ents = ecs.create(num_ents);
    int i = 467831;
    for (const auto& e : ents) {
        ecs.emplace<TestInt>(e, i);
        i ^= (i << 7) ^ (i >> 13) ^ (i << 8);
    }

    for (const auto& e : ents) {
        ecs.remove<TestInt>(e);
    }

    return std::chrono::high_resolution_clock::now() - start;
}


int main() {
    constexpr uint32_t num_iterations = 10000;
    constexpr uint32_t num_entities = 4000;
    std::cout << "Creation and destruction time test: " << num_iterations << " iterations of " << num_entities << " entities..." << std::endl;

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
    std::cout << "\nTesting creating a number of entities" << std::endl;
    const auto ents  = ecs.create(100);
    uint32_t i = 0;
    for (const auto& e : ents) {
        ecs.get_or_emplace<uint32_t>(e, i++);
    }

    mgm::ComponentReference cr{};

    std::cout << "\nTesting component references" << std::endl;
    ecs.emplace<int>(0, 69);
    const auto refs = ecs.get_all(0);
    for (const auto& ct : refs) {
        if (ct.type_id == mgm::MgmEcs::type_id<int>)
            std::cout << "This is an int: " << ct.get<int>();
        else if (ct.type_id == mgm::MgmEcs::type_id<uint32_t>)
            std::cout << "This is a uint32_t: " << ct.get<uint32_t>();
        std::cout << std::endl;
    }

    std::cout << "\nTesting destroying a number of entities" << std::endl;
    const std::vector<mgm::Entity> ents_to_destroy{ents.begin() + 30, ents.begin() + 54};
    ecs.destroy(ents_to_destroy);
    for (const auto& e : ecs.all()) {
        const auto& c = ecs.get<uint32_t>(e);
        std::cout << c << ", ";
    }
    std::cout << std::endl;

    std::cout << "\nTesting creating a number of entities less than available deleted entities" << std::endl;
    auto destroyed_ents = ecs.create(20);
    for (const auto& e : destroyed_ents) {
        ecs.emplace<uint32_t>(e, i++);
    }
    for (const auto& e : ecs.all()) {
        const auto* c = ecs.try_get<uint32_t>(e);
        if (c != nullptr)
            std::cout << *c << ", ";
    }
    std::cout << std::endl;

    std::cout << "\nTesting creating a number of entities more than available entities" << std::endl;
    destroyed_ents = ecs.create(5);
    for (const auto& e : destroyed_ents) {
        ecs.emplace<uint32_t>(e, i++);
    }
    for (const auto& e : ecs.all()) {
        const auto* c = ecs.try_get<uint32_t>(e);
        if (c != nullptr)
            std::cout << *c << ", ";
    }
    std::cout << std::endl;

    std::cout << "\nCausing an intentional crash..." << std::endl;
    try {
        std::cout << "Getting an int reference to an int component... " << refs[mgm::MgmEcs::type_id<int>].get<int>() << std::endl;
        std::cout << "Getting an int reference to a uint32_t component... " << refs[mgm::MgmEcs::type_id<uint32_t>].get<int>() << std::endl;
    } catch (std::runtime_error err) {
        std::cout << "Caught an error: \n\t" << err.what() << std::endl;
    }

    std::cout << "\nAll tests succeded!" << std::endl;
    
    return 0;
}
