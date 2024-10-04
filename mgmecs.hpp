#pragma once
#include <algorithm>
#include <cstdint>
#include <any>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>


namespace mgm {
    template <typename, typename T, typename = void> struct has_on_construct : std::false_type {};
    template <typename C, typename Ret, typename... Args> struct has_on_construct<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_construct(std::declval<Args>()...))>> : std::true_type {};
    #define HAS_CONSTRUCT has_on_construct<T, void(Ecs*, const Entity)>{}

    template <typename, typename T, typename = void> struct has_on_access : std::false_type {};
    template <typename C, typename Ret, typename... Args> struct has_on_access<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_access(std::declval<Args>()...))>> : std::true_type {};
    #define HAS_ACCESS has_on_access<T, void(Ecs*, const Entity)>{}

    template <typename, typename T, typename = void> struct has_on_destroy : std::false_type {};
    template <typename C, typename Ret, typename... Args> struct has_on_destroy<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_destroy(std::declval<Args>()...))>> : std::true_type {};
    #define HAS_DESTROY has_on_destroy<T, void(Ecs*, const Entity)>{}


    template<typename EntityType = uint32_t, bool safety = true>
    class MGMecs {
        using Ecs = MGMecs<EntityType, safety>;

        template<typename T>
        struct TypeHash {
            static constexpr int i{};
            static constexpr const int* value = &i;
        };

    public:
        class Entity {
            EntityType value_;

        public:
            constexpr Entity() : value_(0) {}

            constexpr Entity(EntityType value) : value_(value) {}
            constexpr explicit operator EntityType() const { return value_; }

            constexpr Entity& operator+=(const Entity& other) { value_ += other.value_; return *this; }
            constexpr Entity& operator-=(const Entity& other) { value_ -= other.value_; return *this; }
            constexpr Entity& operator++() { ++value_; return *this; }
            constexpr Entity operator++(int) { return Entity(value_++); }

            // Define more operators as needed
            constexpr Entity operator+(const Entity& other) const { return Entity(value_ + other.value_); }
            constexpr Entity operator-(const Entity& other) const { return Entity(value_ - other.value_); }
            constexpr bool operator==(const Entity& other) const { return value_ == other.value_; }
            constexpr bool operator!=(const Entity& other) const { return value_ != other.value_; }
            constexpr bool operator<(const Entity& other) const { return value_ < other.value_; }
            constexpr bool operator>(const Entity& other) const { return value_ > other.value_; }
            constexpr bool operator<=(const Entity& other) const { return value_ <= other.value_; }
            constexpr bool operator>=(const Entity& other) const { return value_ >= other.value_; }

            constexpr friend std::ostream& operator<<(std::ostream& os, const Entity& e) { return os << e.value_; }
            constexpr friend std::istream& operator>>(std::istream& is, Entity& e) { return is >> e.value_; }

            struct Hash {
                std::size_t operator()(const Entity& e) const { return std::hash<uint32_t>()(e.value_); }
            };
        };
        static constexpr Entity null = static_cast<Entity>(-1);
        static constexpr Entity zero = static_cast<Entity>(0);

        using TypeID = decltype(TypeHash<void>::value);

        template<typename T>
        static constexpr TypeID type_of = TypeID(TypeHash<T>::value);

    private:
        struct GroupContainer {
            virtual void ecs_moved(Ecs* new_location) = 0;
            virtual void add_unreachable(size_t e) = 0;
            virtual ~GroupContainer() = default;
        };

        template<typename T>
        struct ComponentBucket {
            using CompBucket = ComponentBucket<T>;
            std::vector<T> components{};
            std::vector<Entity> original{};
            std::unordered_map<Entity, size_t, typename Entity::Hash> map{};
            std::unordered_set<GroupContainer*> iterating_groups{};

            template<typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& create(Ecs* ecs, const Entity e, Ts&&... args) {
                if constexpr (safety)
                    if (map.find(e) != map.end())
                        throw std::runtime_error("Entity already contains a component of this type");
                map.emplace(e, components.size());
                original.emplace_back(e);
                T& component = components.emplace_back(std::forward<Ts>(args)...);

                if constexpr (HAS_CONSTRUCT)
                    if (ecs != nullptr)
                        component.on_construct(ecs, e);

                return component;
            }

            template<typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            void create(Ecs* ecs, const It& begin, const It& end, Ts&&... args) {
                const auto start = components.size();

                for (auto it = begin; it != end; ++it)
                    create(nullptr, *it, std::forward<Ts>(args)...);

                if constexpr (HAS_CONSTRUCT)
                    if (ecs != nullptr)
                        for (auto i = start; i < components.size(); i++)
                            components[i].on_construct(ecs, original[i]);
            }
            template<typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            void try_create(Ecs* ecs, const It& begin, const It& end, Ts&&... args) {
                const auto start = components.size();

                for (auto it = begin; it != end; ++it)
                    get_or_create(nullptr, *it, std::forward<Ts>(args)...);

                if constexpr (HAS_CONSTRUCT)
                    if (ecs != nullptr)
                        for (auto i = start; i < components.size(); i++)
                            components[i].on_construct(ecs, original[i]);
            }

            const T& get(const Entity e) const {
                if constexpr (safety) {
                    const auto it = map.find(e);
                    if (it == map.end())
                        throw std::runtime_error("Entity does not contain a component of this type");
                    return components[it->second];
                }
                else {
                    const auto c = map.at(e);
                    return components[c];
                }
            }
            T& get(const Entity e) {
                const T& val = const_cast<const CompBucket*>(this)->get(e);
                return const_cast<T&>(val);
            }

            template<typename ... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& get_or_create(Ecs* ecs, const Entity e, Ts&&... args) {
                const auto it = map.find(e);
                if (it == map.end()) {
                    T& component = create(ecs, e, std::forward<Ts>(args)...);

                    return component;
                }
                return components[it->second];
            }

            const T* try_get(const Entity e) const {
                const auto it = map.find(e);
                if (it == map.end())
                    return nullptr;
                return &components[it->second];
            }
            T* try_get(const Entity e) {
                const T* val = const_cast<const CompBucket*>(this)->try_get(e);
                return const_cast<T*>(val);
            }

        private:
            bool __destroy(const size_t c) {
                if (c == components.size() - 1) {
                    components.pop_back();
                    original.pop_back();
                    return true;
                }

                std::swap(components[c], components.back());
                std::swap(original[c], original.back());
                map[original[c]] = c;
                components.pop_back();
                original.pop_back();

                if (!iterating_groups.empty())
                    for (auto group : iterating_groups)
                        group->add_unreachable(c);

                return true;
            }

        public:
            void destroy(Ecs* ecs, const Entity e) {
                const auto it = map.find(e);
                if constexpr (safety)
                    if (it == map.end())
                        throw std::runtime_error("Entity doesn't contain a component of this type");

                const auto c = it->second;

                if constexpr (HAS_DESTROY)
                    if (ecs != nullptr)
                        components[c].on_destroy(ecs, e);

                map.erase(it);
                __destroy(c);
            }
            bool try_destroy(Ecs* ecs, const Entity e) {
                const auto it = map.find(e);
                if (it == map.end())
                    return false;

                const auto c = it->second;

                if constexpr (HAS_DESTROY)
                    if (ecs != nullptr)
                        components[c].on_destroy(ecs, e);

                map.erase(it);
                return __destroy(c);
            }

        private:
            void __destroy_from_to(std::vector<typename decltype(map)::iterator>& to_remove) {
                std::sort(to_remove.begin(), to_remove.end(),
                    [](decltype(map)::iterator a, decltype(map)::iterator b) { return a->second < b->second; }
                );

                for (auto rit = to_remove.rbegin(); rit != to_remove.rend(); ++rit) {
                    const size_t c = (*rit)->second;

                    if (c == components.size() - 1) {
                        components.pop_back();
                        original.pop_back();
                        continue;
                    }

                    std::swap(components[c], components.back());
                    std::swap(original[c], original.back());
                    map[original[c]] = c;
                    components.pop_back();
                    original.pop_back();

                    if (!iterating_groups.empty())
                        for (auto group : iterating_groups)
                            group->add_unreachable(c);
                }
            }

        public:
            template<typename It>
            void destroy(Ecs* ecs, const It& begin, const It& end) {
                if (begin == end)
                    return;

                std::vector<typename decltype(map)::iterator> to_remove;
                to_remove.reserve(std::distance(begin, end));

                for (auto it = begin; it != end; ++it) {
                    const auto entity_it = map.find(*it);
                    if constexpr (safety)
                        if (entity_it == map.end())
                            throw std::runtime_error("Entity doesn't contain a component of this type");
                    to_remove.push_back(entity_it);
                }

                if constexpr (HAS_DESTROY)
                    if (ecs != nullptr)
                        for (const auto& it : to_remove)
                            components[it->second].on_destroy(ecs, it->first);

                for (const auto& it : to_remove)
                    map.erase(it);

                __destroy_from_to(to_remove);
            }
            template<typename It>
            void try_destroy(Ecs* ecs, const It& begin, const It& end) {
                if (begin == end)
                    return;

                std::vector<typename decltype(map)::iterator> to_remove;
                to_remove.reserve(std::distance(begin, end));

                for (auto it = begin; it != end; ++it) {
                    const auto entity_it = map.find(*it);
                    if (entity_it != map.end())
                        to_remove.push_back(entity_it);
                }

                if constexpr (HAS_DESTROY)
                    if (ecs != nullptr)
                        for (const auto& it : to_remove)
                            components[it->second].on_destroy(ecs, it->first);

                for (const auto& it : to_remove)
                    map.erase(it);

                __destroy_from_to(to_remove);
            }

            void destroy_all(Ecs* ecs) {
                if constexpr (HAS_DESTROY) {
                    const auto original_copy = original;
                    destroy(ecs, original_copy.begin(), original_copy.end());
                }
            }
        };

        struct Container {
            std::any bucket{};

            template<typename T>
            Container& init() {
                bucket = ComponentBucket<T>{};
                return *this;
            }

            template<typename T>
            ComponentBucket<T>& get() {
                return std::any_cast<ComponentBucket<T>&>(bucket);
            }
            template<typename T>
            const ComponentBucket<T>& get() const {
                return std::any_cast<const ComponentBucket<T>&>(bucket);
            }

            virtual void destroy(Ecs* ecs, const Entity e) = 0;
            virtual void try_destroy(Ecs* ecs, const Entity e) = 0;
            virtual void try_destroy(Ecs* ecs, const std::vector<Entity>& entities) = 0;
            virtual bool contains(const Entity e) const = 0;
            virtual void destroy_all(Ecs* ecs) = 0;

            virtual ~Container() = default;
        };
        template<typename T>
        Container& get_container() {
            return *containers.at(type_of<T>);
        }
        template<typename T>
        const Container& get_container() const {
            return *containers.at(type_of<T>);
        }

        static inline struct EmptyFunctionalContainer : Container {
            using Container::Container;

            virtual void destroy(Ecs* ecs, const Entity e) override {};
            virtual void try_destroy(Ecs* ecs, const Entity e) override {};
            virtual void try_destroy(Ecs* ecs, const std::vector<Entity>& entities) override {};
            virtual bool contains(const Entity e) const override { return false; }
            virtual void destroy_all(Ecs* ecs) override {};
        } empty_functional_container{};

        template<typename T>
        const Container* get_container_or_empty_functional() const {
            if (container_exists<T>())
                return &get_container<T>();
            return &empty_functional_container;
        }

        template<typename T>
        Container& create_container() {
            struct ContainerT : public Container {
                using Container::Container;

                virtual void destroy(Ecs* ecs, const Entity e) override {
                    auto& bucket = Container::template get<T>();
                    bucket.destroy(ecs, e);
                }
                virtual void try_destroy(Ecs* ecs, const Entity e) override {
                    auto& bucket = Container::template get<T>();
                    if (bucket.try_get(e) == nullptr)
                        return;
                    bucket.destroy(ecs, e);
                }
                virtual void try_destroy(Ecs* ecs, const std::vector<Entity>& entities) override {
                    auto& bucket = Container::template get<T>();
                    bucket.try_destroy(ecs, entities.begin(), entities.end());
                }
                virtual bool contains(const Entity e) const override {
                    auto& bucket = Container::template get<T>();
                    return bucket.try_get(e) != nullptr;
                }
                virtual void destroy_all(Ecs* ecs) override {
                    auto& bucket = Container::template get<T>();
                    bucket.destroy_all(ecs);
                }
            };
            return containers.emplace(type_of<T>, new ContainerT{}).first->second->template init<T>();
        }
        template<typename T>
        Container& get_or_create_container() {
            const auto it = containers.find(type_of<T>);
            if (it == containers.end())
                return create_container<T>();
            return *containers.at(type_of<T>);
        }

        template<typename T>
        void destroy_container() {
            containers.erase(containers.find(type_of<T>));
        }

        template<typename T>
        bool container_exists() const {
            return containers.find(type_of<T>) != containers.end();
        }

        class EntityManager {
            std::unordered_set<Entity, typename Entity::Hash> used{};
            std::priority_queue<Entity, std::vector<Entity>, std::greater<Entity>> available{};
            Entity next = zero;

        public:
            EntityManager() = default;
            EntityManager(const EntityManager&) = default;
            EntityManager(EntityManager&&) = default;
            EntityManager& operator=(const EntityManager&) = default;
            EntityManager& operator=(EntityManager&&) = default;

            Entity create() {
                if (available.empty()) {
                    auto e = *used.emplace(next).first;
                    next = static_cast<Entity>(next + 1);
                    return e;
                }
                const auto id = available.top();
                available.pop();
                return *used.emplace(id).first;
            }

            void destroy(const Entity id) {
                if (id == next - 1) {
                    next = (Entity)(next - 1);
                    while (!available.empty() && available.top() == next - 1) {
                        available.pop();
                        next = (Entity)(next - 1);
                    }
                }
                else if (id < next)
                    available.push(id);

                used.erase(id);
            }
            void try_destroy(const Entity id) {
                if (used.find(id) == used.end())
                    return;
                destroy(id);
            }

            std::unordered_set<Entity>& all() { return used; }
            const std::unordered_set<Entity>& all() const { return used; }

            ~EntityManager() = default;
        };
        EntityManager entities{};
        std::unordered_map<TypeID, Container*> containers{};

    public:
        MGMecs() = default;
        MGMecs(const MGMecs&) = delete;
        MGMecs(MGMecs&& eg) {
            if (this == &eg)
                return;
            entities = std::move(eg.entities);
            containers = std::move(eg.containers);

            for (const auto& group : groups)
                group->ecs_moved(this);
        }
        MGMecs& operator=(const MGMecs&) = delete;
        MGMecs& operator=(MGMecs&& eg) {
            if (this == &eg)
                return *this;
            entities = std::move(eg.entities);
            containers = std::move(eg.containers);

            for (const auto& group : groups)
                group->ecs_moved(this);

            return *this;
        }

        Entity create() {
            return entities.create();
        }

        template<typename It>
        void create(const It& begin, const It& end) {
            for (auto it = begin; it != end; ++it)
                *it = entities.create();
        }

        template<typename T, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        T& emplace(const Entity e, Ts&&... args) {
            auto& bucket = get_or_create_container<T>().template get<T>();
            return bucket.create(this, e, std::forward<Ts>(args)...);
        }
        template<typename T, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        T& get_or_emplace(const Entity e, Ts&&... args) {
            auto& bucket = get_or_create_container<T>().template get<T>();
            return bucket.get_or_create(this, e, std::forward<Ts>(args)...);
        }

        template<typename T, typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        void emplace(const It& begin, const It& end, Ts&&... args) {
            auto& bucket = get_or_create_container<T>().template get<T>();
            bucket.create(this, begin, end, std::forward<Ts>(args)...);
        }
        template<typename T, typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        void try_emplace(const It& begin, const It& end, Ts&&... args) {
            auto& bucket = get_or_create_container<T>().template get<T>();
            bucket.try_create(this, begin, end, std::forward<Ts>(args)...);
        }

        template<typename T>
        T& get(const Entity e) {
            auto& bucket = get_container<T>().template get<T>();
            return bucket.get(e);
        }
        template<typename T>
        const T& get(const Entity e) const {
            const auto& bucket = get_container<T>().template get<T>();
            return bucket.get(e);
        }

        template<typename T>
        T* try_get(const Entity e) {
            if (!container_exists<T>())
                return nullptr;
            auto& bucket = get_container<T>().template get<T>();
            return bucket.try_get(e);
        }
        template<typename T>
        const T* try_get(const Entity e) const {
            if (!container_exists<T>())
                return nullptr;
            const auto& bucket = get_container<T>().template get<T>();
            return bucket.try_get(e);
        }

        template<typename T>
        bool contains(const Entity e) const {
            if (!container_exists<T>())
                return false;
            const auto& bucket = get_container<T>().template get<T>();
            return bucket.try_get(e) != nullptr;
        }

        template<typename T, typename It>
        bool contains(const It& begin, const It& end) {
            if (!container_exists<T>())
                return false;
            const auto& bucket = get_container<T>().template get<T>();
            bool does_contain = true;
            for (auto it = begin; it != end && does_contain; ++it)
                does_contain = does_contain && (bucket.try_get(*it) != nullptr);
            return does_contain;
        }

        template<typename T>
        void remove(const Entity e) {
            auto& bucket = get_container<T>().template get<T>();
            bucket.destroy(this, e);
        }
        template<typename T>
        void try_remove(const Entity e) {
            auto& bucket = get_container<T>().template get<T>();
            if (bucket.try_get(e) == nullptr)
                return;
            bucket.destroy(this, e);
        }

        template<typename T, typename It>
        void remove(const It& begin, const It& end) {
            auto& bucket = get_container<T>().template get<T>();
            bucket.destroy(this, begin, end);
        }
        template<typename T, typename It>
        void try_remove(const It& begin, const It& end) {
            auto& bucket = get_container<T>().template get<T>();
            bucket.try_destroy(this, begin, end);
        }

        void destroy(const Entity e) {
            entities.destroy(e);

            for (auto& [type, container] : containers)
                container->try_destroy(this, e);
        }
        void try_destroy(const Entity e) {
            entities.try_destroy(e);

            for (auto& [type, container] : containers)
                container->try_destroy(this, e);
        }

        template<typename It>
        void destroy(const It& begin, const It& end) {
            std::vector<Entity> entities_to_destroy{};
            entities_to_destroy.reserve(std::distance(begin, end));
            for (auto it = begin; it != end; ++it) {
                entities.destroy(*it);
                entities_to_destroy.push_back(*it);
            }

            for (auto& [type, container] : containers)
                container->try_destroy(this, entities_to_destroy);
        }
        template<typename It>
        void try_destroy(const It& begin, const It& end) {
            std::vector<Entity> entities_to_destroy{};
            entities_to_destroy.reserve(std::distance(begin, end));
            for (auto it = begin; it != end; ++it) {
                entities.try_destroy(*it);
                entities_to_destroy.push_back(*it);
            }

            for (auto& [type, container] : containers)
                container->try_destroy(this, entities_to_destroy);
        }

        template<typename EcsType, typename T = void, typename... Ts>
        struct Group {
        private:
            struct GroupContainerT : public EcsType::GroupContainer {
                Group<EcsType, T, Ts...>* group{};
                GroupContainerT(Group<EcsType, T, Ts...>* group) : group{group} {}

                virtual void ecs_moved(Ecs* new_location) override {
                    group->ecs = new_location;
                }
                virtual void add_unreachable(size_t e) override {
                    for (const auto& iterator : group->iterators)
                        iterator->try_add_unreachable(e);
                }
                virtual ~GroupContainerT() override = default;
            };

        public:
            GroupContainerT* group_container = nullptr;
            EcsType* ecs = nullptr;

            Group() = delete;
            Group(const Group&) = delete;
            Group(Group&&) = delete;
            Group& operator=(const Group&) = delete;
            Group& operator=(Group&&) = delete;

            Group(EcsType& original_ecs) : ecs{&original_ecs} {
                group_container = new GroupContainerT{const_cast<Group<EcsType, T, Ts...>*>(this)};
                const_cast<Ecs*>(ecs)->groups.emplace(group_container);

                auto& bucket = const_cast<Ecs*>(ecs)->template get_container<T>().template get<T>();
                bucket.iterating_groups.emplace(group_container);
            }

            struct Iterator {
                const Group* group = nullptr;
                size_t pos{};
                std::unordered_set<size_t> unreachable{};
                bool dereferenced_last_unreachable = false;

                void try_add_unreachable(size_t c) {
                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    if (pos == bucket.components.size()) {
                        --pos;
                        return;
                    }
                    if (c < pos && (group->ecs->template contains_with_include_exclude<Ts>(bucket.original[c]) && ...)) {
                        unreachable.insert(c);
                        dereferenced_last_unreachable = false;
                    }
                }

                Iterator(const Group* group, size_t pos) : group(group), pos(pos) {
                    if (group != nullptr)
                        const_cast<Group*>(group)->iterators.emplace(this);
                }
                Iterator() = delete;
                Iterator(const Iterator& other) {
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr)
                        const_cast<Group*>(group)->iterators.emplace(this);
                }
                Iterator(Iterator&& other) {
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        const_cast<Group*>(group)->iterators.erase(&other);
                        const_cast<Group*>(group)->iterators.emplace(this);
                    }
                }
                Iterator& operator=(const Iterator& other) {
                    if (this == &other)
                        return *this;
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr)
                        const_cast<Group*>(group)->iterators.emplace(this);
                    return *this;
                }
                Iterator& operator=(Iterator&& other) {
                    if (this == &other)
                        return *this;
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        const_cast<Group*>(group)->iterators.erase(&other);
                        const_cast<Group*>(group)->iterators.emplace(this);
                    }
                    return *this;
                }

                template<typename EcsTypeT = EcsType, std::enable_if_t<std::is_const_v<EcsTypeT>, bool> = true>
                std::pair<Entity, const T&> operator*() {
                    if (group == nullptr)
                        throw std::runtime_error("Dereferencing an invalid or end iterator");

                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    if (!unreachable.empty()) {
                        const auto component = *unreachable.begin();
                        dereferenced_last_unreachable = true;
                        return std::pair<Entity, const T&>{bucket.original[component], bucket.components[component]};
                    }
                    return std::pair<Entity, const T&>{bucket.original[pos], bucket.components[pos]};
                }
                template<typename EcsTypeT = EcsType, std::enable_if_t<!std::is_const_v<EcsTypeT>, bool> = true>
                std::pair<Entity, T&> operator*() {
                    if (group == nullptr)
                        throw std::runtime_error("Dereferencing an invalid or end iterator");

                    auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    if (!unreachable.empty()) {
                        const auto component = *unreachable.begin();
                        dereferenced_last_unreachable = true;
                        return std::pair<Entity, T&>{bucket.original[component], bucket.components[component]};
                    }
                    return std::pair<Entity, T&>{bucket.original[pos], bucket.components[pos]};
                }

                Iterator& operator++() {
                    if (group == nullptr)
                        throw std::runtime_error("Incrementing an invalid or end iterator");

                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();

                    if (!unreachable.empty()) {
                        if (!dereferenced_last_unreachable)
                            return *this;

                        do {
                            unreachable.erase(unreachable.begin());
                        }
                        while (!unreachable.empty() && !(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...));
                        dereferenced_last_unreachable = false;
                        return ++*this;
                    }

                    do {
                        ++pos;
                    }
                    while (!(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...));

                    if (pos >= bucket.components.size()) {
                        const_cast<Group*>(group)->iterators.erase(this);
                        group = nullptr;
                        pos = 0;
                        return *this;
                    }

                    return *this;
                }
                Iterator operator++(int) {
                    Iterator res{*this};
                    ++(*this);
                    return res;
                }

                Iterator& operator+=(size_t n) {
                    for (size_t i = 0; i < n && group != nullptr; ++i)
                        ++(*this);
                    return *this;
                }
                Iterator operator+(size_t n) const {
                    Iterator res{*this};
                    res += n;
                    return res;
                }

                bool operator==(const Iterator& other) const {
                    return pos == other.pos && group == other.group;
                }
                bool operator!=(const Iterator& other) const {
                    return !(*this == other);
                }

                ~Iterator() {
                    if (group != nullptr)
                        const_cast<Group*>(group)->iterators.erase(this);
                }
            };

            private:
                std::unordered_set<Iterator*> iterators{};

            public:

            Iterator begin() const {
                const auto& bucket = ecs->template get_container<T>().template get<T>();
                size_t pos = 0;
                while (!(ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...) && (pos < bucket.components.size()))
                    ++pos;
                return Iterator{this, pos};
            }
            Iterator end() const { return Iterator{nullptr, 0}; }

            ~Group() {
                auto& bucket = const_cast<Ecs*>(ecs)->template get_container<T>().template get<T>();
                bucket.iterating_groups.erase(group_container);

                const_cast<Ecs*>(ecs)->groups.erase(ecs->groups.find(group_container));
                delete group_container;
            }
        };

    private:
        std::unordered_set<GroupContainer*> groups{};

        template <class T, template <class...> class Template>
        struct is_specialization : std::false_type {};

        template <template <class...> class Template, class... Args>
        struct is_specialization<Template<Args...>, Template> : std::true_type {};

        template <typename T>
        struct get_specialization_type;

        template <template <typename> class Template, typename T>
        struct get_specialization_type<Template<T>> {
            using type = T;
        };

        template <typename T>
        using get_specialization_type_t = typename get_specialization_type<T>::type;

        template<typename IE>
        bool contains_with_include_exclude(const Entity e) const {
            using T = get_specialization_type_t<IE>;
            const auto contains = get_container_or_empty_functional<T>()->contains(e);
            return (contains && is_specialization<IE, Exclude>{}) || (!contains && is_specialization<IE, Include>{});
        }

    public:
        template<typename T> struct Include{};
        template<typename T> struct Exclude{};

        template<typename T, typename... Ts>
        Group<Ecs, T, Ts...> group() {
            if (!container_exists<T>() || (((!container_exists<get_specialization_type_t<Ts>>() && is_specialization<Ts, Include>{}) &&...) && sizeof...(Ts) > 0))
                return Group<Ecs, T, Ts...>{*this};

            return Group<Ecs, T, Ts...>{*this};
        }

        template<typename T, typename... Ts>
        Group<const Ecs, T, Ts...> group() const {
            if (!container_exists<T>() || (((!container_exists<get_specialization_type_t<Ts>>() && is_specialization<Ts, Include>{}) &&...) && sizeof...(Ts) > 0))
                return Group<const Ecs, T, Ts...>{*this};

            return Group<const Ecs, T, Ts...>{*this};
        }

        ~MGMecs() {
            while (!containers.empty()) {
                const auto container = containers.begin();

                container->second->destroy_all(this);
                delete container->second;
                containers.erase(container);
            }
        }
    };
}


namespace std {
    template<>
    struct hash<const int* const> {
        std::size_t operator()(const int* const ptr) const {
            return std::hash<std::uintptr_t>()(reinterpret_cast<std::uintptr_t>(ptr));
        }
    };
}
