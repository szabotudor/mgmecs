#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <condition_variable>


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

        mutable std::recursive_mutex mutex{};

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
            virtual void new_added() = 0;
            virtual ~GroupContainer() = default;
        };

        template<typename T>
        struct ComponentBucket {
            mutable std::recursive_mutex mutex{};

            std::vector<T> components{};
            std::vector<Entity> original{};
            std::unordered_map<Entity, size_t, typename Entity::Hash> map{};

            struct ThreadSafeSet {
                std::unordered_set<size_t> set{};
                std::mutex mutex{};
                std::condition_variable cv{};

                void insert(size_t e) {
                    std::unique_lock lock{mutex};
                    set.insert(e);
                    cv.notify_all();
                }
                void erase(size_t e) {
                    std::unique_lock lock{mutex};
                    set.erase(e);
                    cv.notify_all();
                }
                bool contains(size_t e) {
                    std::unique_lock lock{mutex};
                    return set.find(e) != set.end();
                }
                void wait_without_lock_on(size_t e) {
                    std::unique_lock lock{mutex};
                    cv.wait(lock, [this, e] { return set.find(e) != set.end(); });
                }
                void lock_on(size_t e) {
                    std::unique_lock lock{mutex};
                    if (set.find(e) != set.end())
                        cv.wait(lock, [this, e] { return set.find(e) != set.end(); });
                    set.insert(e);
                }
                void clear() {
                    std::unique_lock lock{mutex};
                    set.clear();
                    cv.notify_all();
                }
                ~ThreadSafeSet() = default;
            } busy_components{};
            
            std::unordered_set<GroupContainer*> iterating_groups{};

            ComponentBucket() = default;

            bool empty() const {
                std::unique_lock lock{mutex};
                return components.empty();
            }

            template<typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& create(Ecs* ecs, const Entity e, Ts&&... args) {
                std::unique_lock lock{mutex};
                if constexpr (safety)
                    if (map.find(e) != map.end())
                        throw std::runtime_error("Entity already contains a component of this type");
                map.emplace(e, components.size());
                original.emplace_back(e);
                T& component = components.emplace_back(std::forward<Ts>(args)...);
                const auto c = components.size() - 1;
                if constexpr (HAS_CONSTRUCT)
                    if (ecs != nullptr)
                        busy_components.insert(c);

                for (auto group : iterating_groups)
                    group->new_added();

                lock.unlock();

                if constexpr (HAS_CONSTRUCT) {
                    if (ecs != nullptr) {
                        component.on_construct(ecs, e);
                        busy_components.erase(c);
                    }
                }

                return component;
            }

            template<typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            void create(Ecs* ecs, const It& begin, const It& end, Ts&&... args) {
                std::vector<std::pair<T*, size_t>> constructed{};
                constructed.reserve(std::distance(begin, end));

                std::unique_lock lock{mutex};

                for (auto it = begin; it != end; ++it) {
                    if constexpr (safety)
                        if (map.find(*it) != map.end())
                            throw std::runtime_error("Entity already contains a component of this type");
                    map.emplace(*it, components.size());
                    original.emplace_back(*it);
                    constructed.emplace_back(&components.emplace_back(std::forward<Ts>(args)...), components.size() - 1);
                    busy_components.insert(components.size() - 1);
                }

                for (auto group : iterating_groups)
                    group->new_added();

                lock.unlock();

                if constexpr (HAS_CONSTRUCT) {
                    if (ecs != nullptr) {
                        for (const auto& [component, c] : constructed) {
                            component->on_construct(ecs, original[c]);
                            busy_components.erase(c);
                        }
                    }
                }
            }
            template<typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            void try_create(Ecs* ecs, const It& begin, const It& end, Ts&&... args) {
                std::vector<std::pair<T*, size_t>> constructed{};
                constructed.reserve(std::distance(begin, end));

                std::unique_lock lock{mutex};

                const auto original_size = original.size();

                for (auto it = begin; it != end; ++it) {
                    if (map.find(*it) != map.end())
                        continue;
                    map.emplace(*it, components.size());
                    original.emplace_back(*it);
                    constructed.emplace_back(&components.emplace_back(std::forward<Ts>(args)...), components.size() - 1);
                    busy_components.insert(components.size() - 1);
                }

                if (original.size() > original_size)
                    for (auto group : iterating_groups)
                        group->new_added();

                lock.unlock();
                
                if constexpr (HAS_CONSTRUCT) {
                    if (ecs != nullptr) {
                        for (const auto& [component, c] : constructed) {
                            component->on_construct(ecs, original[c]);
                            busy_components.erase(c);
                        }
                    }
                }
            }

            const T& get(const Entity e) const {
                std::unique_lock lock{mutex};
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
                std::unique_lock lock{mutex};
                const T& val = const_cast<const ComponentBucket<T>*>(this)->get(e);
                return const_cast<T&>(val);
            }

            template<typename ... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& get_or_create(Ecs* ecs, const Entity e, Ts&&... args) {
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if (it == map.end()) {
                    lock.unlock();
                    return create(ecs, e, std::forward<Ts>(args)...);
                }
                return components[it->second];
            }

            const T* try_get(const Entity e) const {
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if (it == map.end())
                    return nullptr;
                return &components[it->second];
            }
            T* try_get(const Entity e) {
                const T* val = const_cast<const ComponentBucket<T>*>(this)->try_get(e);
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
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if constexpr (safety)
                    if (it == map.end())
                        throw std::runtime_error("Entity doesn't contain a component of this type");

                busy_components.lock_on(it->second);
                lock.unlock();

                const auto c = it->second;

                if constexpr (HAS_DESTROY) {
                    if (ecs != nullptr) {
                        components[c].on_destroy(ecs, e);
                        busy_components.erase(c);
                    }
                }

                lock.lock();
                map.erase(it);
                __destroy(c);
            }
            bool try_destroy(Ecs* ecs, const Entity e) {
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if (it == map.end())
                    return false;

                busy_components.lock_on(it->second);
                lock.unlock();

                const auto c = it->second;

                if constexpr (HAS_DESTROY) {
                    if (ecs != nullptr) {
                        components[c].on_destroy(ecs, e);
                        busy_components.erase(c);
                    }
                }

                lock.lock();
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

                std::unique_lock lock{mutex};

                for (auto it = begin; it != end; ++it) {
                    const auto entity_it = map.find(*it);
                    if constexpr (safety)
                        if (entity_it == map.end())
                            throw std::runtime_error("Entity doesn't contain a component of this type");
                    busy_components.lock_on(entity_it->second);
                    to_remove.push_back(entity_it);
                }

                lock.unlock();

                if constexpr (HAS_DESTROY) {
                    if (ecs != nullptr) {
                        for (const auto& it : to_remove) {
                            components[it->second].on_destroy(ecs, it->first);
                            busy_components.erase(it->second);
                        }
                    }
                }

                lock.lock();
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

                std::unique_lock lock{mutex};

                for (auto it = begin; it != end; ++it) {
                    const auto entity_it = map.find(*it);
                    if (entity_it != map.end()) {
                        busy_components.lock_on(entity_it->second);
                        to_remove.push_back(entity_it);
                    }
                }

                lock.unlock();

                if constexpr (HAS_DESTROY) {
                    if (ecs != nullptr) {
                        for (const auto& it : to_remove) {
                            components[it->second].on_destroy(ecs, it->first);
                            busy_components.erase(it->second);
                        }
                    }
                }

                lock.lock();
                for (const auto& it : to_remove)
                    map.erase(it);

                __destroy_from_to(to_remove);
            }

            void destroy_all(Ecs* ecs) {
                const auto original_copy = original;
                destroy(ecs, original_copy.begin(), original_copy.end());
            }
        };

        struct Container {
            struct Any {
                void* value = nullptr;
                std::function<void(void*)> destructor = nullptr;
                
                template<typename T, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
                void emplace(Ts&&... args) {
                    value = new T(std::forward<Ts>(args)...);
                    destructor = [](void* ptr) { delete static_cast<T*>(ptr); };
                }

                template<typename T>
                T& get() {
                    return *static_cast<T*>(value);
                }
                template<typename T>
                const T& get() const {
                    return *static_cast<T*>(value);
                }

                void destroy() {
                    if (destructor != nullptr)
                        destructor(value);
                    value = nullptr;
                    destructor = nullptr;
                }

                ~Any() {
                    destroy();
                }
            } bucket{};

            template<typename T>
            Container& init() {
                bucket.template emplace<ComponentBucket<T>>();
                return *this;
            }

            template<typename T>
            ComponentBucket<T>& get() {
                return bucket.template get<ComponentBucket<T>>();
            }
            template<typename T>
            const ComponentBucket<T>& get() const {
                return bucket.template get<ComponentBucket<T>>();
            }

            virtual bool empty() const = 0;
            virtual void destroy(Ecs* ecs, const Entity e) = 0;
            virtual void try_destroy(Ecs* ecs, const Entity e) = 0;
            virtual void try_destroy(Ecs* ecs, const std::vector<Entity>& entities) = 0;
            virtual bool contains(const Entity e) const = 0;
            virtual void destroy_all(Ecs* ecs) = 0;

            virtual ~Container() = default;
        };
        template<typename T>
        Container& get_container() {
            std::unique_lock lock{mutex};
            return *containers.at(type_of<T>);
        }
        template<typename T>
        const Container& get_container() const {
            std::unique_lock lock{mutex};
            return *containers.at(type_of<T>);
        }

        template<typename T>
        Container* try_get_container() {
            std::unique_lock lock{mutex};
            const auto it = containers.find(type_of<T>);
            if (it == containers.end())
                return nullptr;
            return it->second;
        }
        template<typename T>
        const Container* try_get_container() const {
            std::unique_lock lock{mutex};
            const auto it = containers.find(type_of<T>);
            if (it == containers.end())
                return nullptr;
            return it->second;
        }

        static inline struct EmptyFunctionalContainer : Container {
            using Container::Container;

            virtual bool empty() const override { return true; }
            virtual void destroy(Ecs* ecs, const Entity e) override {};
            virtual void try_destroy(Ecs* ecs, const Entity e) override {};
            virtual void try_destroy(Ecs* ecs, const std::vector<Entity>& entities) override {};
            virtual bool contains(const Entity e) const override { return false; }
            virtual void destroy_all(Ecs* ecs) override {};
        } empty_functional_container{};

        template<typename T>
        const Container* get_container_or_empty_functional() const {
            std::unique_lock lock{mutex};
            if (container_exists<T>())
                return &get_container<T>();
            return &empty_functional_container;
        }

        template<typename T>
        Container& create_container() {
            std::unique_lock lock{mutex};
            struct ContainerT : public Container {
                using Container::Container;

                virtual bool empty() const override {
                    auto& bucket = Container::template get<T>();
                    return bucket.empty();
                }
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
            std::unique_lock lock{mutex};
            const auto it = containers.find(type_of<T>);
            if (it == containers.end())
                return create_container<T>();
            return *containers.at(type_of<T>);
        }

        template<typename T>
        void destroy_container() {
            std::unique_lock lock{mutex};
            containers.erase(containers.find(type_of<T>));
        }

        template<typename T>
        bool container_exists() const {
            std::unique_lock lock{mutex};
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
            bool try_destroy(const Entity id) {
                if (used.find(id) == used.end())
                    return false;
                destroy(id);
                return true;
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
        MGMecs(MGMecs&& other) {
            if (this == &other)
                return;

            std::unique_lock lock{mutex};
            std::unique_lock other_lock{other.mutex};

            entities = std::move(other.entities);
            containers = std::move(other.containers);
            groups = std::move(other.groups);

            for (const auto& group : groups)
                group->ecs_moved(this);
        }
        MGMecs& operator=(const MGMecs&) = delete;
        MGMecs& operator=(MGMecs&& other) {
            if (this == &other)
                return *this;

            std::unique_lock lock{mutex};
            std::unique_lock other_lock{other.mutex};

            entities = std::move(other.entities);
            containers = std::move(other.containers);
            groups = std::move(other.groups);

            for (const auto& group : groups)
                group->ecs_moved(this);

            return *this;
        }

        Entity create() {
            std::unique_lock lock{mutex};
            return entities.create();
        }

        template<typename It>
        void create(const It& begin, const It& end) {
            std::unique_lock lock{mutex};
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
            auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return nullptr;
            return bucket->try_get(e);
        }
        template<typename T>
        const T* try_get(const Entity e) const {
            const auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return nullptr;
            return bucket->try_get(e);
        }

        template<typename T>
        bool contains(const Entity e) const {
            const auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return false;
            return bucket->try_get(e) != nullptr;
        }

        template<typename T, typename It>
        bool contains(const It& begin, const It& end) {
            const auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return false;
            bool does_contain = true;
            for (auto it = begin; it != end && does_contain; ++it)
                does_contain = does_contain && (bucket->try_get(*it) != nullptr);
            return does_contain;
        }

        template<typename T>
        void remove(const Entity e) {
            auto& bucket = get_container<T>().template get<T>();
            bucket.destroy(this, e);
        }
        template<typename T>
        void try_remove(const Entity e) {
            auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return;
            bucket->try_destroy(this, e);
        }

        template<typename T, typename It>
        void remove(const It& begin, const It& end) {
            auto& bucket = get_container<T>().template get<T>();
            bucket.destroy(this, begin, end);
        }
        template<typename T, typename It>
        void try_remove(const It& begin, const It& end) {
            auto* bucket = try_get_container<T>().template get<T>();
            if (bucket == nullptr)
                return;
            bucket->try_destroy(this, begin, end);
        }

        void destroy(const Entity e) {
            std::unique_lock lock{mutex};
            entities.destroy(e);
            std::vector<Container*> to_destroy{};
            to_destroy.reserve(containers.size());
            for (auto& [type, container] : containers)
                if (container->contains(e))
                    to_destroy.push_back(container);
            lock.unlock();

            for (auto& container : to_destroy)
                container->destroy(this, e);
        }
        void try_destroy(const Entity e) {
            std::unique_lock lock{mutex};
            if (!entities.try_destroy(e))
                return;
            std::vector<Container*> to_destroy{};
            to_destroy.reserve(containers.size());
            for (auto& [type, container] : containers)
                if (container->contains(e))
                    to_destroy.push_back(container);
            lock.unlock();

            for (auto& container : to_destroy)
                container->try_destroy(this, e);
        }

        template<typename It>
        void destroy(const It& begin, const It& end) {
            std::vector<Entity> entities_to_destroy{};
            std::vector<Container*> to_destroy{};

            std::unique_lock lock{mutex};
            entities_to_destroy.reserve(std::distance(begin, end));
            for (auto it = begin; it != end; ++it) {
                entities.destroy(*it);
                entities_to_destroy.push_back(*it);
            }

            to_destroy.reserve(containers.size());
            for (auto& [type, container] : containers)
                to_destroy.push_back(container);

            lock.unlock();

            for (auto& container : to_destroy)
                container->try_destroy(this, entities_to_destroy);
        }
        template<typename It>
        void try_destroy(const It& begin, const It& end) {
            std::vector<Entity> entities_to_destroy{};
            std::vector<Container*> to_destroy{};

            std::unique_lock lock{mutex};
            entities_to_destroy.reserve(std::distance(begin, end));
            bool any_destroyed = false;
            for (auto it = begin; it != end; ++it) {
                if (entities.try_destroy(*it)) {
                    any_destroyed = true;
                    entities_to_destroy.push_back(*it);
                }
            }

            if (!any_destroyed)
                return;

            to_destroy.reserve(containers.size());
            for (auto& [type, container] : containers)
                to_destroy.push_back(container);

            lock.unlock();

            for (auto& container : to_destroy)
                container->try_destroy(this, entities_to_destroy);
        }

        template<typename EcsType, typename T = void, typename... Ts>
        struct Group {
        private:
            mutable std::recursive_mutex mutex{};
            bool is_reversed = false;

            struct GroupContainerT : public EcsType::GroupContainer {
                Group<EcsType, T, Ts...>* group{};
                GroupContainerT(Group<EcsType, T, Ts...>* group) : group{group} {}

                virtual void ecs_moved(Ecs* new_location) override {
                    std::unique_lock lock{group->mutex};
                    group->ecs = new_location;
                }
                virtual void add_unreachable(size_t c) override {
                    std::unique_lock lock{group->mutex};
                    for (const auto& iterator : group->iterators)
                        iterator->try_add_unreachable(c);
                }
                virtual void new_added() override {
                    std::unique_lock lock{group->mutex};
                    for (const auto& iterator : group->iterators)
                        iterator->try_new_added();
                }
                virtual ~GroupContainerT() override = default;
            };

        public:
            GroupContainerT* group_container = nullptr;
            EcsType* ecs = nullptr;

            Group() = delete;
            Group(const Group& other) {
                new (this) Group{*other.ecs};
            }
            Group(Group&& other) {
                ecs = other.ecs;
                group_container = other.group_container;
                other.group_container = nullptr;
                other.ecs = nullptr;
            }
            Group& operator=(const Group other) {
                if (this == &other)
                    return *this;
                destroy_self();
                new (this) Group{other.ecs};
            }
            Group& operator=(Group&& other) {
                if (this == &other)
                    return *this;
                destroy_self();
                ecs = other.ecs;
                group_container = other.group_container;
                other.group_container = nullptr;
                other.ecs = nullptr;
            }

            Group(EcsType& original_ecs) : ecs{&original_ecs} {
                group_container = new GroupContainerT{const_cast<Group<EcsType, T, Ts...>*>(this)};
                std::unique_lock lock{ecs->mutex};
                ecs->groups.emplace(group_container);
                lock.unlock();

                auto& bucket = ecs->template get_container<T>().template get<T>();

                std::unique_lock bucket_lock{bucket.mutex};
                bucket.iterating_groups.emplace(group_container);
            }

            struct Iterator {
                const Group* group = nullptr;
                size_t pos{};
                std::unordered_set<size_t> unreachable{};
                bool dereferenced_last_unreachable: 1 = false;
                bool is_end: 1 = false;
                bool is_reversed: 1 = false;

                void try_add_unreachable(size_t c) {
                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    std::unique_lock lock{bucket.mutex};
                    if (pos >= bucket.components.size()) {
                        pos = bucket.components.size();
                        return;
                    }
                    if (c < pos && (group->ecs->template contains_with_include_exclude<Ts>(bucket.original[c]) && ...)) {
                        unreachable.insert(c);
                        dereferenced_last_unreachable = false;
                    }
                }
                void try_new_added() {
                    if (!is_end)
                        return;
                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    std::unique_lock lock{bucket.mutex};
                    if (pos < bucket.components.size()) {
                        pos = bucket.components.size();
                        return;
                    }
                }

                Iterator(const Group* group, size_t pos, bool is_end = false, bool reverse = false) : group(group), pos(pos), is_end(is_end), is_reversed(reverse) {
                    if (group != nullptr)
                        group->iterators.emplace(this);
                }
                Iterator() = delete;
                Iterator(const Iterator& other) {
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        std::unique_lock lock{group->mutex};
                        group->iterators.emplace(this);
                    }
                }
                Iterator(Iterator&& other) {
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        std::unique_lock lock{group->mutex};
                        group->iterators.erase(&other);
                        group->iterators.emplace(this);
                    }
                }
                Iterator& operator=(const Iterator& other) {
                    if (this == &other)
                        return *this;
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        std::unique_lock lock{group->mutex};
                        group->iterators.emplace(this);
                    }
                    return *this;
                }
                Iterator& operator=(Iterator&& other) {
                    if (this == &other)
                        return *this;
                    group = other.group;
                    pos = other.pos;
                    if (group != nullptr) {
                        std::unique_lock lock{group->mutex};
                        group->iterators.erase(&other);
                        group->iterators.emplace(this);
                    }
                    return *this;
                }

                template<typename EcsTypeT = EcsType, std::enable_if_t<std::is_const_v<EcsTypeT>, bool> = true>
                std::pair<Entity, const T&> operator*() {
                    if (group == nullptr)
                        throw std::runtime_error("Dereferencing an invalid iterator, or owning group has been destroyed");

                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    std::unique_lock lock{bucket.mutex};
                    if (!unreachable.empty() && !is_reversed) {
                        const auto component = *unreachable.begin();
                        dereferenced_last_unreachable = true;
                        return std::pair<Entity, const T&>{bucket.original[component], bucket.components[component]};
                    }
                    if (pos >= bucket.components.size())
                        throw std::runtime_error("Dereferencing an invalid iterator, or owning group has been destroyed");
                    return std::pair<Entity, const T&>{bucket.original[pos], bucket.components[pos]};
                }
                template<typename EcsTypeT = EcsType, std::enable_if_t<!std::is_const_v<EcsTypeT>, bool> = true>
                std::pair<Entity, T&> operator*() {
                    if (group == nullptr)
                        throw std::runtime_error("Dereferencing an invalid iterator, or owning group has been destroyed");

                    auto& bucket = group->ecs->template get_container<T>().template get<T>();
                    std::unique_lock lock{bucket.mutex};
                    if (!unreachable.empty() && !is_reversed) {
                        const auto component = *unreachable.begin();
                        dereferenced_last_unreachable = true;
                        return std::pair<Entity, T&>{bucket.original[component], bucket.components[component]};
                    }
                    if (pos >= bucket.components.size())
                        throw std::runtime_error("Dereferencing an invalid iterator, or owning group has been destroyed");
                    return std::pair<Entity, T&>{bucket.original[pos], bucket.components[pos]};
                }

            private:
                Iterator& inc() {
                    if (group == nullptr)
                        throw std::runtime_error("Incrementing an invalid iterator, or owning group has been destroyed");

                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();

                    std::unique_lock lock{bucket.mutex};

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

                    if (pos == static_cast<size_t>(-1)) {
                        pos = 0;
                        while (!(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...))
                            ++pos;
                        return *this;
                    }

                    if (pos >= bucket.components.size()) {
                        is_end = true;
                        pos = bucket.components.size();
                        return *this;
                    }

                    do {
                        ++pos;
                    }
                    while (!(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...) && (pos < bucket.components.size()));

                    return *this;
                }
                Iterator& dec() {
                    if (group == nullptr)
                        throw std::runtime_error("Decrementing an invalid iterator, or owning group has been destroyed");
                    
                    if (is_end)
                        is_end = false;

                    const auto& bucket = group->ecs->template get_container<T>().template get<T>();

                    std::unique_lock lock{bucket.mutex};

                    do {
                        if (pos == 0 || pos == static_cast<size_t>(-1)) {
                            pos = static_cast<size_t>(-1);
                            is_end = true;
                            return *this;
                        }

                        do {
                            --pos;
                        }
                        while (!(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...) && (pos > 0));

                        if (pos == 0 && !(group->ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...)) {
                            pos = static_cast<size_t>(-1);
                            return *this;
                        }
                    }
                    while (!unreachable.empty() && unreachable.find(pos) != unreachable.end());

                    return *this;
                }

            public:
                Iterator& operator++() {
                    if (is_reversed)
                        return dec();
                    return inc();
                }
                Iterator operator++(int) {
                    Iterator res{*this};
                    ++(*this);
                    return res;
                }

                Iterator& operator--() {
                    if (is_reversed)
                        return inc();
                    return dec();
                }
                Iterator operator--(int) {
                    Iterator res{*this};
                    --(*this);
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

                Iterator& operator-=(size_t n) {
                    for (size_t i = 0; i < n && group != nullptr; ++i)
                        --(*this);
                    return *this;
                }
                Iterator operator-(size_t n) const {
                    Iterator res{*this};
                    res -= n;
                    return res;
                }

                bool operator==(const Iterator& other) const {
                    return (pos == other.pos && group == other.group) || (is_end && other.is_end);
                }
                bool operator!=(const Iterator& other) const {
                    return !(*this == other);
                }

                ~Iterator() {
                    if (group != nullptr) {
                        std::unique_lock lock{group->mutex};
                        const_cast<Group*>(group)->iterators.erase(this);
                    }
                }
            };

            private:
                mutable std::unordered_set<Iterator*> iterators{};

                Iterator true_begin() const {
                    std::unique_lock lock{mutex};
                    const auto& bucket = ecs->template get_container<T>().template get<T>();
                    size_t pos = 0;
                    std::unique_lock bucket_lock{bucket.mutex};
                    while (!(ecs->template contains_with_include_exclude<Ts>(bucket.original[pos]) && ...) && (pos < bucket.components.size()))
                        ++pos;
                    return Iterator{this, pos};
                }
                Iterator true_end() const {
                    std::unique_lock lock{mutex};
                    const auto& bucket = ecs->template get_container<T>().template get<T>();
                    std::unique_lock bucket_lock{bucket.mutex};
                    return Iterator{this, bucket.components.size(), true};
                }

            public:

            Iterator begin() const {
                Iterator it = is_reversed ? true_end() - 1 : true_begin();
                it.is_reversed = is_reversed;
                return it;
            }
            Iterator end() const {
                Iterator it = is_reversed ? true_begin() - 1 : true_end();
                it.is_reversed = is_reversed;
                return it;
            }

            Iterator rbegin() const {
                Iterator it = is_reversed ? true_begin() : true_end() - 1;
                it.is_reversed = !is_reversed;
                return it;
            }
            Iterator rend() const {
                Iterator it = is_reversed ? true_end() : true_begin() - 1;
                it.is_reversed = !is_reversed;
                return it;
            }

            Group<EcsType, T, Ts...> reverse() const {
                std::unique_lock lock{mutex};
                Group copy{*this};
                copy.is_reversed = !is_reversed;
                return copy;
            }

            void destroy_self() {
                std::unique_lock lock{mutex};
                for (const auto& iterator : iterators) {
                    iterator->group = nullptr;
                    iterator->pos = 0;
                }
                iterators.clear();

                if (ecs == nullptr)
                    return;

                auto& bucket = const_cast<Ecs*>(ecs)->template get_container<T>().template get<T>();
                bucket.iterating_groups.erase(group_container);

                const_cast<Ecs*>(ecs)->groups.erase(ecs->groups.find(group_container));
                delete group_container;

                ecs = nullptr;
                group_container = nullptr;
            }

            ~Group() {
                destroy_self();
            }
        };

    private:
        mutable std::unordered_set<GroupContainer*> groups{};

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
                const auto it = containers.begin();
                const auto container = it->second;

                while (!container->empty())
                    container->destroy_all(this);
                delete container;
                containers.erase(it);
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
