#pragma once
#include <algorithm>
#include <any>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>


namespace mgm {
    template<typename, typename T, typename = void>
    struct has_on_construct : std::false_type {};
    template<typename C, typename Ret, typename... Args>
    struct has_on_construct<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_construct(std::declval<Args>()...))>> : std::true_type {};
#define HAS_CONSTRUCT \
    has_on_construct<T, void(Ecs*, const Entity)> {}

    template<typename, typename T, typename = void>
    struct has_on_access : std::false_type {};
    template<typename C, typename Ret, typename... Args>
    struct has_on_access<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_access(std::declval<Args>()...))>> : std::true_type {};
#define HAS_ACCESS \
    has_on_access<T, void(Ecs*, const Entity)> {}

    template<typename, typename T, typename = void>
    struct has_on_destroy : std::false_type {};
    template<typename C, typename Ret, typename... Args>
    struct has_on_destroy<C, Ret(Args...), std::void_t<decltype(std::declval<C>().on_destroy(std::declval<Args>()...))>> : std::true_type {};
#define HAS_DESTROY \
    has_on_destroy<T, void(Ecs*, const Entity)> {}

    template<typename Key, typename T, size_t max_size = 255>
    class MGMecsConstexprMap {
      public:
        struct Item {
            Key key{};
            T val{};
        };

        Item items[max_size]{};
        size_t size = 0;

      private:
        template<typename U>
        consteval void _init1(U&& arg) {
            const auto& [location, item] = arg;
            items[size].key = location;
            items[size].val = item;
            ++size;
        }

      public:
        template<typename... Ts, std::enable_if_t<(std::is_same_v<Ts, Item> && ...), bool> = true>
        consteval MGMecsConstexprMap(Ts&&... args) {
            (_init1(args), ...);
        }

        consteval auto operator[](const Key& key) const {
            for (const auto& [i_key, item] : items)
                if (i_key == key)
                    return item;
            throw "Key doesn't exist in map";
        }

        consteval const Item* begin() const { return items; }
        consteval const Item* end() const { return items + size; }
    };


    template<typename EntityType = uint32_t>
    class MGMecs {
        using Ecs = MGMecs<EntityType>;

        mutable std::recursive_mutex mutex{};

      public:
        class Entity {
          public:
            EntityType value_;

            using Type = EntityType;

            constexpr Entity()
                : value_(null.value_) {}

            constexpr Entity(EntityType value)
                : value_(value) {}

            constexpr explicit operator EntityType() const { return value_; }

            constexpr Entity& operator+=(const Entity& other) {
                value_ += other.value_;
                return *this;
            }
            constexpr Entity& operator-=(const Entity& other) {
                value_ -= other.value_;
                return *this;
            }
            constexpr Entity& operator++() {
                ++value_;
                return *this;
            }
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
                std::size_t operator()(const Entity& e) const { return std::hash<EntityType>()(e.value_); }
            };
        };

        static constexpr Entity null = static_cast<Entity>(static_cast<EntityType>(-1));
        static constexpr Entity zero = static_cast<Entity>(0);

        // WTF
        // And thank you to this stack overflow poster: https://stackoverflow.com/a/77093119/10978039
        template<auto Id>
        struct TIDCounter {
            struct TIDGenerator {
                template<auto>
                friend consteval auto is_defined(TIDCounter<Id>) { return true; }
            };
            template<auto>
            friend consteval auto is_defined(TIDCounter<Id>);

            template<typename Tag = TIDCounter<Id>, auto = is_defined<0>(Tag{})>
            static consteval auto exists(auto) { return true; }

            static consteval auto exists(...) {
                TIDGenerator();
                return false;
            }
        };
        template<typename T>
        struct TypeID {
            template<typename U = T, auto Id = size_t{}>
            static inline consteval auto type_id() {
                if constexpr (TIDCounter<Id>::exists(Id)) {
                    return type_id<T, Id + 1>();
                }
                else {
                    return Id;
                }
            }

            static inline constexpr size_t _id = type_id<T>();

            inline constexpr operator size_t() const { return _id; }
        };

        struct EntityLock {
            Ecs* ecs = nullptr;
            Entity e{};
            std::unordered_set<Entity, typename Entity::Hash> es{};
            bool is_locked = false;

            EntityLock() = default;

            EntityLock(Ecs& base_ecs, Entity entity, bool lock_after_construct = true)
                : ecs(&base_ecs),
                  e(entity) {
                std::unique_lock ecs_lock{ecs->mutex};
                ecs->s_locks.insert(this);
                ecs_lock.unlock();
                if (lock_after_construct)
                    lock();
            }

            template<typename It>
            EntityLock(Ecs& base_ecs, const It& begin, const It& end, bool lock_after_construct = true)
                : ecs(&base_ecs),
                  e(null),
                  es{begin, end} {
                std::unique_lock ecs_lock{ecs->mutex};
                ecs->s_locks.insert(this);
                ecs_lock.unlock();
                if (lock_after_construct)
                    lock();
            }

            EntityLock(const EntityLock& other)
                : ecs(other.ecs) {
                if (!other.is_valid())
                    return;

                std::unique_lock lock{ecs->mutex};
                e = other.e;
                es = other.es;
                is_locked = other.is_locked;

                ecs->s_locks.insert(this);
            }
            EntityLock(EntityLock&& other)
                : ecs(std::move(other.ecs)),
                  e(std::move(other.e)),
                  es(std::move(other.es)),
                  is_locked(std::move(other.is_locked)) {
                other.ecs = nullptr;

                if (!is_valid())
                    return;

                ecs->s_locks.erase(&other);
                ecs->s_locks.insert(this);
            }
            EntityLock& operator=(const EntityLock& other) {
                if (this == &other)
                    return *this;

                invalidate();

                ecs = other.ecs;
                if (!other.is_valid())
                    return *this;

                std::unique_lock lock{ecs->mutex};
                e = other.e;
                es = other.es;
                is_locked = other.is_locked;

                ecs->s_locks.insert(this);

                return *this;
            }
            EntityLock& operator=(EntityLock&& other) {
                if (this == &other)
                    return *this;

                invalidate();

                ecs = other.ecs;
                e = other.e;
                es = std::move(other.es);
                is_locked = other.is_locked;

                other.ecs = nullptr;

                ecs->s_locks.erase(&other);
                ecs->s_locks.insert(this);

                return *this;
            }

            void lock() {
                if (!is_valid())
                    throw std::runtime_error("Trying to lock an invalid lock");
                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null)
                    ecs->wait_and_lock(e);
                else
                    ecs->wait_and_lock(es.begin(), es.end());
                is_locked = true;
            }
            void unlock() {
                if (!is_valid())
                    throw std::runtime_error("Trying to unlock an invalid lock");
                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null)
                    ecs->unlock(e);
                else
                    ecs->unlock(es.begin(), es.end());
                is_locked = false;
            }

            void append(Entity entity) {
                if (!is_valid())
                    throw std::runtime_error("Trying to append an entity to an invalid lock");

                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null) {
                    es.insert(e);
                    es.insert(entity);
                    e = null;
                    if (is_locked)
                        ecs->wait_and_lock(entity);
                }
                else
                    e = entity;
            }
            template<typename It>
            void append(const It& begin, const It& end) {
                if (!is_valid())
                    throw std::runtime_error("Trying to append entities to an invalid lock");

                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null) {
                    es.insert(begin, end);
                    const auto _e = e;
                    if (is_locked) {
                        e = null;
                        lock();
                    }
                    es.insert(_e);
                }
                else {
                    const auto _es = std::move(es);
                    es.clear();
                    es.reserve(_es.size() + std::distance(begin, end));
                    es.insert(begin, end);
                    if (is_locked)
                        lock();
                    es.insert(_es.begin(), _es.end());
                }
            }

            void remove(Entity entity) {
                if (!is_valid())
                    throw std::runtime_error("Trying to unlock an entity from an invalid lock");

                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null) {
                    if (e != entity)
                        throw std::runtime_error("Trying to remove an entity that isn't part of this lock");

                    invalidate();
                }
                else {
                    const auto it = es.find(entity);
                    if (it == es.end())
                        throw std::runtime_error("Trying to remove an entity that isn't part of this lock");
                    if (is_locked)
                        ecs->unlock(*it);
                    es.erase(it);
                }
            }
            template<typename It>
            void remove(const It& begin, const It& end) {
                if (!is_valid())
                    throw std::runtime_error("Trying to unlock entities from an invalid lock");

                const auto dist = std::distance(begin, end);

                const std::unique_lock ecs_lock{ecs->mutex};
                if (e != null) {
                    if (dist != 1)
                        throw std::runtime_error("Trying to remove more entities than available in the lock");

                    const auto _e = *begin;
                    if (e != _e)
                        throw std::runtime_error("Trying to remove an entity that isn't part of this lock");

                    invalidate();
                }
                else {
                    std::vector<Entity> to_remove{};
                    to_remove.reserve(dist);

                    for (auto it = begin; it != end; ++it) {
                        const auto _it = es.find(*it);
                        if (_it == es.end())
                            throw std::runtime_error("Trying to remove and entity that isn't part of this lock");
                        to_remove.emplace_back(*it);
                        es.erase(_it);
                    }

                    if (is_locked)
                        ecs->unlock(to_remove.begin(), to_remove.end());
                }
            }

            bool is_valid() { return ecs != nullptr; }

            void invalidate() {
                if (!is_valid())
                    return;

                if (is_locked)
                    unlock();

                std::unique_lock ecs_lock{ecs->mutex};
                ecs->s_locks.erase(this);
                ecs_lock.unlock();
                ecs = nullptr;
                e = null;
                es = {};
            }

            ~EntityLock() {
                invalidate();
            }
        };

      private:
        template<typename T, typename Hash = std::hash<T>>
        struct ThreadSafeSet {
            friend class MGMecs<EntityType>;

            struct LockStatus {
                std::thread::id thread_id{};
                size_t count = 0;
                bool disallow_further_locking = false;

                LockStatus()
                    : thread_id(std::this_thread::get_id()),
                      count(1),
                      disallow_further_locking(false) {}
            };
            std::unordered_map<T, LockStatus, Hash> map{};
            std::mutex mutex{};
            std::condition_variable cv{};
            bool allow_locking = true;

          private:
            void _unlock(const T& e) {
                const auto it = map.find(e);
                if (it == map.end())
                    throw std::runtime_error("Trying to unlock an entity that was never locked");
                else if (it->second.thread_id == std::this_thread::get_id()) {
                    if (--it->second.count == 0) {
                        if (it->second.disallow_further_locking)
                            it->second.thread_id = {};
                        else
                            map.erase(e);
                    }
                }
                else
                    throw std::runtime_error("Trying to unlock an entity that was locked by a different thread");
            }

          public:
            void unlock(const T& e) {
                {
                    std::unique_lock lock{mutex};
                    _unlock(e);
                }
                cv.notify_all();
            }
            template<typename It>
            void unlock(const It& begin, const It& end) {
                {
                    std::unique_lock lock{mutex};
                    for (auto it = begin; it != end; ++it)
                        _unlock(*it);
                }
                cv.notify_all();
            }

          private:
            void _wait_and_lock(const T& e) {
                const auto it = map.find(e);
                if (it == map.end())
                    map.emplace(e, LockStatus{});
                else if (it->second.thread_id == std::this_thread::get_id())
                    ++it->second.count;
                else
                    throw std::runtime_error("INTERNAL ERROR: Trying to lock an entity that is already locked by another thread");
            }

          public:
            void wait_and_lock(const T& e) {
                {
                    std::unique_lock lock{mutex};
                    if (!allow_locking || map.find(e) != map.end())
                        cv.wait(lock, [this, e] {
                            if (!allow_locking)
                                return false;

                            const auto it = map.find(e);
                            if (it == map.end())
                                return true;
                            return it->second.thread_id == std::this_thread::get_id();
                        });
                    _wait_and_lock(e);
                }
                cv.notify_all();
            }
            template<typename It>
            void wait_and_lock(const It& begin, const It& end) {
                std::unique_lock lock{mutex};
                cv.wait(lock, [&]() {
                    return allow_locking;
                });

                bool any_locked = false;
                for (auto it = begin; it != end; ++it) {
                    if (map.find(*it) != map.end()) {
                        any_locked = true;
                        break;
                    }
                }
                allow_locking = false;

                if (any_locked) {
                    cv.wait(lock, [this, begin, end] {
                        bool any_still_locked = false;
                        for (auto e_it = begin; e_it != end; ++e_it) {
                            const auto it = map.find(*e_it);
                            if (it != map.end()) {
                                if (it->second.thread_id != std::this_thread::get_id()) {
                                    any_still_locked = true;
                                    break;
                                }
                            }
                        }
                        return !any_still_locked;
                    });
                }

                for (auto it = begin; it != end; ++it)
                    _wait_and_lock(*it);

                allow_locking = true;

                lock.unlock();
                cv.notify_all();
            }

            ~ThreadSafeSet() = default;
        };

        template<typename T>
        struct GenericIteratorHelper {
            using iterator_category = std::bidirectional_iterator_tag;
            using value_type = T;
            using difference_type = size_t;
            using pointer = T*;
            using reference = T&;

            std::any base{};
            std::function<void(GenericIteratorHelper* gih)> inc{};
            std::function<void(GenericIteratorHelper* gih)> dec{};
            std::function<T*(GenericIteratorHelper* gih)> deref{};
            std::function<bool(const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2)> cmp{};
            std::function<difference_type(const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2)> dif{};

            template<typename U, std::enable_if_t<!std::is_same_v<U, GenericIteratorHelper<T>>, bool> = true>
            GenericIteratorHelper(U& base_iterator)
                : base{base_iterator},
                  inc([](GenericIteratorHelper* gih) {
                      U& base_data = std::any_cast<U&>(gih->base);
                      ++base_data;
                  }),
                  dec([](GenericIteratorHelper* gih) {
                      U& base_data = std::any_cast<U&>(gih->base);
                      --base_data;
                  }),
                  deref([](GenericIteratorHelper* gih) -> T* {
                      U& base_data = std::any_cast<U&>(gih->base);
                      return &*base_data;
                  }),
                  cmp([](const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2) {
                      U& base_data1 = std::any_cast<U&>(gih1->base);
                      U& base_data2 = std::any_cast<U&>(gih2->base);
                      return base_data1 == base_data2;
                  }),
                  dif([](const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2) -> difference_type {
                      U& base_data1 = std::any_cast<U&>(gih1->base);
                      U& base_data2 = std::any_cast<U&>(gih2->base);
                      return base_data1 - base_data2;
                  }) {}
            template<typename U, std::enable_if_t<!std::is_same_v<U, GenericIteratorHelper<T>>, bool> = true>
            GenericIteratorHelper(const U& base_iterator)
                : base{base_iterator},
                  inc([](GenericIteratorHelper* gih) {
                      const U& base_data = std::any_cast<const U&>(gih->base);
                      gih->base = base_data + 1;
                  }),
                  dec([](GenericIteratorHelper* gih) {
                      const U& base_data = std::any_cast<const U&>(gih->base);
                      gih->base = base_data + 1;
                  }),
                  deref([](GenericIteratorHelper* gih) -> T* {
                      const U& base_data = std::any_cast<const U&>(gih->base);
                      return const_cast<T*>(&*base_data);
                  }),
                  cmp([](const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2) {
                      const U& base_data1 = std::any_cast<const U&>(gih1->base);
                      const U& base_data2 = std::any_cast<const U&>(gih2->base);
                      return base_data1 == base_data2;
                  }),
                  dif([](const GenericIteratorHelper* gih1, const GenericIteratorHelper* gih2) -> difference_type {
                      const U& base_data1 = std::any_cast<const U&>(gih1->base);
                      const U& base_data2 = std::any_cast<const U&>(gih2->base);
                      return base_data1 - base_data2;
                  }) {}

            GenericIteratorHelper() = delete;

            GenericIteratorHelper(const GenericIteratorHelper&) = default;
            GenericIteratorHelper(GenericIteratorHelper&&) = default;
            GenericIteratorHelper& operator=(const GenericIteratorHelper&) = default;
            GenericIteratorHelper& operator=(GenericIteratorHelper&&) = default;

            T& operator*() {
                return *deref(this);
            }
            const T& operator*() const {
                return *deref(const_cast<GenericIteratorHelper*>(this));
            }

            T* operator->() {
                return deref(this);
            }
            const T* operator->() const {
                return deref(const_cast<GenericIteratorHelper*>(this));
            }

            GenericIteratorHelper& operator++() {
                inc(this);
                return *this;
            }
            GenericIteratorHelper& operator--() {
                dec(this);
                return *this;
            }
            GenericIteratorHelper& operator++(int) {
                auto cpy = *this;
                inc(this);
                return cpy;
            }
            GenericIteratorHelper& operator--(int) {
                auto cpy = *this;
                dec(this);
                return cpy;
            }

            bool operator==(const GenericIteratorHelper& other) const {
                return cmp(this, &other);
            }
            bool operator!=(const GenericIteratorHelper& other) const {
                return !cmp(this, &other);
            }

            size_t operator-(const GenericIteratorHelper& other) const {
                return dif(this, &other);
            }
        };

        struct Container {
            Container() = default;

            virtual size_t count() const = 0;
            virtual bool try_destroy(Ecs* ecs, const Entity e) = 0;
            virtual bool try_destroy(Ecs* ecs, const GenericIteratorHelper<Entity>& begin, const GenericIteratorHelper<Entity>& end) = 0;
            virtual void destroy_all(Ecs* ecs) = 0;

            virtual ~Container() = default;
        };

        template<typename Key, typename Callback, typename KeyHash = std::hash<Key>>
        class CallbacksHelper {};

        template<typename Key, typename Ret, typename... Args, typename KeyHash>
        class CallbacksHelper<Key, Ret(Args...), KeyHash> {
            using Callback = Ret(Args...);
            using CBH = CallbacksHelper<Key, Callback, KeyHash>;
            using CB = std::function<Callback>;

            struct Collection {
                std::unordered_map<size_t, CB> callbacks{};
                size_t p = 0;
            };

            std::unordered_map<Key, Collection, KeyHash> callbacks;
            bool currently_calling = false;

            struct ToDestroy {
                Key key{};
                size_t p{};
            };
            std::vector<ToDestroy> to_destroy_after_calls{};

          public:
            CallbacksHelper() = default;

            CallbacksHelper(const CallbacksHelper&) = delete;
            CallbacksHelper(CallbacksHelper&&) = delete;
            CallbacksHelper operator=(const CallbacksHelper&) = delete;
            CallbacksHelper operator=(CallbacksHelper&&) = delete;

            struct CallbackHandle {
                CBH* helper = nullptr;
                Key key{};
                size_t p = 0;

                CallbackHandle() = default;

                CallbackHandle(const CallbackHandle&) = delete;
                CallbackHandle(CallbackHandle&& other)
                    : helper(std::move(other.helper)),
                      key(std::move(other.key)),
                      p{std::move(other.p)} {
                    other.helper = nullptr;
                    other.key = {};
                    other.p = 0;
                }
                CallbackHandle& operator=(const CallbackHandle&) = delete;
                CallbackHandle& operator=(CallbackHandle&& other) {
                    if (this == &other)
                        return *this;

                    invalidate();

                    helper = std::move(other.helper);
                    key = std::move(other.key);
                    p = std::move(other.p);
                    other.helper = nullptr;
                    other.key = {};
                    other.p = 0;

                    return *this;
                }

                CallbackHandle(CBH& callbacks_helper, const Key& callback_key, size_t collection_p)
                    : helper(&callbacks_helper),
                      key(callback_key),
                      p{collection_p} {}

                void invalidate() {
                    if (helper) {
                        if (helper->currently_calling)
                            helper->to_destroy_after_calls.emplace_back(ToDestroy{.key = key, .p = p});
                        else {
                            helper->callbacks.at(key).callbacks.erase(p);
                            helper = nullptr;
                            p = 0;
                        }
                    }
                }

                ~CallbackHandle() {
                    invalidate();
                }
            };

            [[nodiscard]] CallbackHandle create(const Key& key, const CB& callback) {
                auto it = callbacks.find(key);

                if (it == callbacks.end())
                    it = callbacks.emplace(key, Collection{}).first;

                const auto p = it->second.p++;
                it->second.callbacks.emplace(p, callback);

                return CallbackHandle{*this, key, p};
            }

            template<typename ReturnType = Ret, std::enable_if_t<!std::is_same_v<ReturnType, void>, bool> = true>
            std::vector<Ret> operator()(Key key, Args&&... args) {
                std::vector<Ret> res{};
                res.reserve(callbacks.size());

                const auto it = callbacks.find(key);
                if (it == callbacks.end())
                    return {};

                if (currently_calling)
                    throw std::runtime_error("INTERNAL ERROR: Don't call the callback manager from inside of a callback already being executed");

                currently_calling = true;

                for (const auto& [p, cb] : it->second.callbacks)
                    res.emplace_back(cb(std::forward<Args>(args)...));

                currently_calling = false;
                for (const auto& [k, p] : to_destroy_after_calls)
                    callbacks.at(k).callbacks.erase(p);
                to_destroy_after_calls.clear();

                return res;
            }

            template<typename ReturnType = Ret, std::enable_if_t<std::is_same_v<ReturnType, void>, bool> = true>
            void operator()(Key key, Args&&... args) {
                const auto it = callbacks.find(key);
                if (it == callbacks.end())
                    return;

                currently_calling = true;

                for (const auto& [p, cb] : it->second.callbacks)
                    cb(std::forward<Args>(args)...);

                currently_calling = false;
                for (const auto& [k, p] : to_destroy_after_calls)
                    callbacks.at(k).callbacks.erase(p);
                to_destroy_after_calls.clear();
            }
        };

        template<typename T>
        struct ComponentBucket : public Container {
            mutable std::recursive_mutex mutex{};

            struct Component {
                bool latent_destruction : 1 = false;
                EntityType c : (sizeof(EntityType) * 8 - 1) = 0;

                Component() = default;

                operator EntityType() const {
                    return static_cast<EntityType>(c);
                }

                Component(const size_t base)
                    : c(base & ((EntityType(1) << (sizeof(EntityType) * 8 - 1)) - 1)) {}

                template<std::enable_if_t<!std::is_same_v<EntityType, size_t>, bool> = true>
                Component(const size_t base)
                    : c(static_cast<EntityType>(base) & ((EntityType(1) << (sizeof(EntityType) * 8 - 1)) - 1)) {}

                friend Component operator+(const EntityType& s, const Component& comp) { return Component{s + comp.c}; }
                friend Component operator-(const EntityType& s, const Component& comp) { return Component{s - comp.c}; }
                friend Component operator*(const EntityType& s, const Component& comp) { return Component{s * comp.c}; }
                friend Component operator/(const EntityType& s, const Component& comp) { return Component{s / comp.c}; }

                Component operator+(const Component& other) { return {c + other.c}; }
                Component operator-(const Component& other) { return {c - other.c}; }
                Component operator*(const Component& other) { return {c * other.c}; }
                Component operator/(const Component& other) { return {c / other.c}; }

                Component& operator+=(const Component& other) {
                    c += other.c;
                    return *this;
                }
                Component& operator-=(const Component& other) {
                    c -= other.c;
                    return *this;
                }
                Component& operator*=(const Component& other) {
                    c *= other.c;
                    return *this;
                }
                Component& operator/=(const Component& other) {
                    c /= other.c;
                    return *this;
                }
            };

            std::vector<T> components{};
            std::vector<Entity> original{};
            std::unordered_map<Entity, Component, typename Entity::Hash> map{};

            std::unordered_map<EntityType, T*> latent_destruction_components{};
            EntityType ldc_id_p = EntityType(0);

            using CompMoveCallbacks = CallbacksHelper<Entity, void(ComponentBucket<T>& originating_bucket, size_t from, size_t to), typename Entity::Hash>;
            CompMoveCallbacks comp_move_callbacks{};

            ComponentBucket() = default;

            size_t count() const override {
                std::unique_lock lock{mutex};
                return components.size() + latent_destruction_components.size();
            }

          private:
            const T* _get(const Component& c) const {
                if (c.latent_destruction) {
                    const auto it = latent_destruction_components.find(c.c);
                    if (it == latent_destruction_components.end())
                        return nullptr;
                    return it->second;
                }
                return &components[c.c];
            }
            T* _get(const Component& c) {
                return const_cast<T*>(const_cast<const ComponentBucket<T>*>(this)->_get(c));
            }

          public:
            template<typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& create(Ecs* ecs, const Entity e, Ts&&... args) {
                std::unique_lock lock{mutex};

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
            bool try_create(Ecs* ecs, const It& begin, const It& end, Ts&&... args) {
                std::unique_lock lock{mutex};

                std::vector<std::pair<T*, Component>> constructed{};
                constructed.reserve(std::distance(begin, end));

                const auto original_size = original.size();

                for (auto it = begin; it != end; ++it) {
                    if (map.find(*it) != map.end())
                        continue;
                    map.emplace(*it, components.size());
                    original.emplace_back(*it);
                    constructed.emplace_back(&components.emplace_back(std::forward<Ts>(args)...), components.size() - 1);
                }

                lock.unlock();

                if constexpr (HAS_CONSTRUCT)
                    if (ecs != nullptr)
                        for (const auto& [component, c] : constructed)
                            component->on_construct(ecs, original[c]);
                return true;
            }

            const T& get(const Entity e) const {
                std::unique_lock lock{mutex};
                const auto c = map.at(e);
                const auto component = _get(c);
                if (component == nullptr)
                    throw std::runtime_error("INTERNAL ERROR: Invalid component ID related to entity");
                return *component;
            }
            T& get(const Entity e) {
                const T& val = const_cast<const ComponentBucket<T>*>(this)->get(e);
                return const_cast<T&>(val);
            }

            template<typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
            T& get_or_create(Ecs* ecs, const Entity e, Ts&&... args) {
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if (it == map.end()) {
                    lock.unlock();
                    return create(ecs, e, std::forward<Ts>(args)...);
                }
                const auto component = _get(it->second);
                if (component == nullptr)
                    throw std::runtime_error("INTERNAL ERROR: Invalid component ID related to entity");
                return *component;
            }

            const T* try_get(const Entity e) const {
                std::unique_lock lock{mutex};
                const auto it = map.find(e);
                if (it == map.end())
                    return nullptr;
                const auto component = _get(it->second);
                if (component == nullptr)
                    throw std::runtime_error("INTERNAL ERROR: Invalid component ID related to entity");
                return component;
            }
            T* try_get(const Entity e) {
                const T* val = const_cast<const ComponentBucket<T>*>(this)->try_get(e);
                return const_cast<T*>(val);
            }

          private:
            bool _destroy(Ecs* ecs, const Component c) {
                if (ecs == nullptr)
                    return false;

                if (c.c == components.size() - 1) {
                    const auto o = original.back();
                    components.pop_back();
                    original.pop_back();
                    comp_move_callbacks(o, *this, original.size(), static_cast<size_t>(-1));
                    return true;
                }

                const auto o = original.back();
                std::swap(components[c.c], components.back());
                std::swap(original[c.c], original.back());
                map[original[c.c]] = c;
                components.pop_back();
                original.pop_back();

                comp_move_callbacks(o, *this, original.size(), c.c);

                return true;
            }

          public:
            bool try_destroy(Ecs* ecs, const Entity e) override {
                if (ecs == nullptr)
                    return false;

                std::unique_lock lock{mutex};

                const auto it = map.find(e);
                if (it == map.end())
                    return false;

                Component& c = it->second;

                if constexpr (HAS_DESTROY) {
                    T temp{std::move(components[c.c])};
                    _destroy(ecs, c);

                    const auto ldc = ldc_id_p++;
                    latent_destruction_components.emplace(ldc, &temp);
                    c = ldc;
                    c.latent_destruction = true;

                    lock.unlock();
                    temp.on_destroy(ecs, e);
                    lock.lock();

                    map.erase(e);
                    latent_destruction_components.erase(ldc);
                    return true;
                }
                else {
                    _destroy(ecs, c);
                    map.erase(e);
                    return true;
                }
            }

            template<typename It>
            bool try_destroy(Ecs* ecs, const It& begin, const It& end, bool abort_on_invalid = false) {
                if (ecs == nullptr)
                    return false;

                const auto dist = std::distance(begin, end);

                std::unique_lock lock{mutex};

                static thread_local std::vector<Component*> to_delete{};
                if (static_cast<decltype(dist)>(to_delete.capacity()) < dist)
                    to_delete.reserve(dist);
                to_delete.clear();

                for (auto it = begin; it != end; ++it) {
                    const auto e = *it;
                    const auto mit = map.find(e);
                    if (mit != map.end())
                        to_delete.emplace_back(&mit->second);
                    else if (abort_on_invalid)
                        return false;
                }

                if constexpr (HAS_DESTROY) {
                    struct Temp {
                        T t;
                        Entity e;
                        EntityType ldc;
                        Temp(T&& _t, Entity _e, EntityType _ldc)
                            : t(_t),
                              e(_e),
                              ldc(_ldc) {}
                    };
                    std::vector<Temp> temps{};
                    temps.reserve(to_delete.size());
                    for (const auto c : to_delete) {
                        const auto ldc = ldc_id_p++;
                        temps.emplace_back(std::move(components[c->c]), original[c->c], ldc);
                        _destroy(ecs, *c);
                        latent_destruction_components.emplace(ldc, &temps.back().t);
                        *c = ldc;
                        c->latent_destruction = true;
                    }

                    lock.unlock();
                    for (auto& [t, e, ldc] : temps)
                        t.on_destroy(ecs, e);
                    lock.lock();

                    for (const auto& [t, e, ldc] : temps) {
                        map.erase(e);
                        latent_destruction_components.erase(ldc);
                    }
                    return true;
                }
                else {
                    for (const auto c : to_delete) {
                        const auto e = original[c->c];
                        _destroy(ecs, *c);
                        map.erase(e);
                    }
                    return true;
                }
            }

            bool try_destroy(Ecs* ecs, const GenericIteratorHelper<Entity>& begin, const GenericIteratorHelper<Entity>& end) override {
                return try_destroy<GenericIteratorHelper<Entity>>(ecs, begin, end);
            }

            void destroy_all(Ecs* ecs) override {
                std::unique_lock lock{mutex};
                const auto original_copy = original;
                lock.unlock();

                try_destroy(ecs, original_copy.begin(), original_copy.end());
            }
        };

        template<typename T>
        ComponentBucket<T>& get_bucket() {
            std::unique_lock lock{mutex};
            return *reinterpret_cast<ComponentBucket<T>*>(buckets.at(TypeID<T>{}));
        }
        template<typename T>
        const ComponentBucket<T>& get_bucket() const {
            std::unique_lock lock{mutex};
            return *reinterpret_cast<ComponentBucket<T>*>(buckets.at(TypeID<T>{}));
        }

        template<typename T>
        ComponentBucket<T>* try_get_bucket() {
            std::unique_lock lock{mutex};
            const auto it = buckets.find(TypeID<T>{});
            if (it == buckets.end())
                return nullptr;
            return reinterpret_cast<ComponentBucket<T>*>(it->second);
        }
        template<typename T>
        const ComponentBucket<T>* try_get_bucket() const {
            std::unique_lock lock{mutex};
            const auto it = buckets.find(TypeID<T>{});
            if (it == buckets.end())
                return nullptr;
            return reinterpret_cast<const ComponentBucket<T>*>(it->second);
        }

        template<typename T>
        ComponentBucket<T>& create_bucket() {
            std::unique_lock lock{mutex};
            const auto c = new ComponentBucket<T>{};
            buckets.emplace(TypeID<T>{}, c);
            return *c;
        }
        template<typename T>
        ComponentBucket<T>& get_or_create_bucket() {
            std::unique_lock lock{mutex};
            const auto it = buckets.find(TypeID<T>{});
            if (it == buckets.end())
                return create_bucket<T>();
            return *reinterpret_cast<ComponentBucket<T>*>(it->second);
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

            std::unordered_set<Entity, typename Entity::Hash>& all() { return used; }
            const std::unordered_set<Entity, typename Entity::Hash>& all() const { return used; }

            ~EntityManager() = default;
        };
        EntityManager entities{};
        std::unordered_map<size_t, Container*> buckets{};
        ThreadSafeSet<Entity, typename Entity::Hash> locks{};

      public:
        MGMecs() = default;
        MGMecs(const MGMecs&) = delete;
        MGMecs(MGMecs&& other) noexcept {
            if (this == &other)
                return;

            std::unique_lock lock{mutex};
            std::unique_lock other_lock{other.mutex};

            for (const auto& g : other.groups)
                g->ecs_moved(this);
            for (const auto& l : other.s_locks)
                l.ecs = this;

            groups = std::move(other.groups);
            entities = std::move(other.entities);
            buckets = std::move(other.buckets);
            locks = std::move(other.locks);
            s_locks = std::move(other.s_locks);
        }
        MGMecs& operator=(const MGMecs&) = delete;
        MGMecs& operator=(MGMecs&& other) noexcept {
            if (this == &other)
                return *this;

            std::unique_lock lock{mutex};
            std::unique_lock other_lock{other.mutex};

            for (const auto& g : other.groups)
                g->ecs_moved(this);
            for (const auto& l : other.s_locks)
                l.ecs = this;

            groups = std::move(other.groups);
            entities = std::move(other.entities);
            buckets = std::move(other.buckets);
            locks = std::move(other.locks);
            s_locks = std::move(other.s_locks);

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
            auto& bucket = get_or_create_bucket<T>();
            return bucket.create(this, e, std::forward<Ts>(args)...);
        }
        template<typename T, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        T& get_or_emplace(const Entity e, Ts&&... args) {
            auto& bucket = get_or_create_bucket<T>();
            return bucket.get_or_create(this, e, std::forward<Ts>(args)...);
        }

        template<typename T, typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        void emplace(const It& begin, const It& end, Ts&&... args) {
            auto& bucket = get_or_create_bucket<T>();
            if (!bucket.try_create(this, begin, end, std::forward<Ts>(args)...))
                throw std::runtime_error("Could not emplace a component on one of the entities");
        }
        template<typename T, typename It, typename... Ts, std::enable_if_t<std::is_constructible_v<T, Ts...>, bool> = true>
        void try_emplace(const It& begin, const It& end, Ts&&... args) {
            auto& bucket = get_or_create_bucket<T>();
            bucket.try_create(this, begin, end, std::forward<Ts>(args)...);
        }

        template<typename T>
        T& get(const Entity e) {
            auto& bucket = get_bucket<T>();
            return bucket.get(e);
        }
        template<typename T>
        const T& get(const Entity e) const {
            const auto& bucket = get_bucket<T>();
            return bucket.get(e);
        }

        template<typename T>
        T* try_get(const Entity e) {
            auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return nullptr;

            return bucket->try_get(e);
        }
        template<typename T>
        const T* try_get(const Entity e) const {
            const auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return nullptr;
            return bucket->try_get(e);
        }

        template<typename T>
        bool contains(const Entity e) const {
            return try_get<T>(e) != nullptr;
        }

        template<typename T, typename It>
        bool contains(const It& begin, const It& end) {
            const auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return false;
            bool does_contain = true;
            for (auto it = begin; it != end && does_contain; ++it)
                does_contain = does_contain && (bucket->try_get(*it) != nullptr);
            return does_contain;
        }

        template<typename T>
        void remove(const Entity e) {
            auto& bucket = get_bucket<T>();
            locks.wait_and_lock(e);
            if (!bucket.try_destroy(this, e))
                throw std::runtime_error("Could not remove component from entity");
            locks.unlock(e);
        }
        template<typename T>
        void try_remove(const Entity e) {
            auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return;

            locks.wait_and_lock(e);
            bucket->try_destroy(this, e);
            locks.unlock(e);
        }

        template<typename T, typename It>
        void remove(const It& begin, const It& end) {
            auto& bucket = get_bucket<T>();
            locks.wait_and_lock(begin, end);
            bucket.try_destroy(this, begin, end);
            locks.unlock(begin, end);
        }
        template<typename T, typename It>
        void try_remove(const It& begin, const It& end) {
            auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return;
            locks.wait_and_lock(begin, end);
            bucket->try_destroy(this, begin, end);
            locks.unlock(begin, end);
        }

        void destroy(const Entity e) {
            locks.wait_and_lock(e);

            std::unique_lock lock{mutex};
            entities.destroy(e);

            for (auto& [type, bucket] : buckets)
                bucket->try_destroy(this, e);

            locks.unlock(e);
        }
        void try_destroy(const Entity e) {
            std::unique_lock lock{mutex};
            locks.wait_and_lock(e);

            if (!entities.try_destroy(e))
                return;

            for (auto& [type, bucket] : buckets)
                bucket->try_destroy(this, e);

            locks.unlock(e);
        }

        template<typename It>
        void destroy(const It& begin, const It& end) {
            const auto dist = std::distance(begin, end);
            if (dist == 0)
                return;

            std::unique_lock lock{mutex};

            locks.wait_and_lock(begin, end);

            for (auto it = begin; it != end; ++it)
                entities.destroy(*it);

            for (auto& [type, bucket] : buckets)
                bucket->try_destroy(this, GenericIteratorHelper<Entity>{begin}, GenericIteratorHelper<Entity>{end});

            locks.unlock(begin, end);
        }
        template<typename It>
        void try_destroy(const It& begin, const It& end) {
            std::unique_lock lock{mutex};
            bool any_destroyed = false;

            locks.wait_and_lock(begin, end);

            for (auto it = begin; it != end; ++it)
                if (entities.try_destroy(*it))
                    any_destroyed = true;

            if (!any_destroyed) {
                locks.unlock(begin, end);
                return;
            }

            for (auto& [type, bucket] : buckets)
                bucket->try_destroy(this, GenericIteratorHelper<Entity>{begin}, GenericIteratorHelper<Entity>{end});

            locks.unlock(begin, end);
        }

        template<typename T>
        Entity as_entity(const T& component) const {
            const auto* bucket = try_get_bucket<T>();
            if (bucket == nullptr)
                return null;

            std::unique_lock lock{bucket->mutex};

            const auto* first = bucket->components.data();
            const auto* last = first + bucket->components.size();

            if (&component < first || &component >= last)
                return null;

            return bucket->original[&component - first];
        }

        EntityType entities_count() const {
            std::unique_lock lock{mutex};
            return static_cast<EntityType>(entities.all().size());
        }

      private:
        std::unordered_set<EntityLock*> s_locks{};

      public:
        [[nodiscard]] EntityLock entity_lock(const Entity e) {
            return EntityLock{*this, e};
        }
        template<typename It>
        [[nodiscard]] EntityLock entity_lock(const It& begin, const It& end) {
            return EntityLock{*this, begin, end};
        }

      private:
        struct GroupContainer {
            virtual void ecs_moved(const Ecs* new_loc) = 0;
        };

        std::unordered_set<GroupContainer*> groups{};

        template<typename T, typename... Ts>
        struct _TypeList {
            using First = T;
        };
        template<typename... Ts>
        struct TypeList {
            using First = typename _TypeList<Ts...>::First;
        };

      public:
        using GroupCond = std::function<bool(Ecs* ecs, const Entity entity)>;

        template<typename EcsType = Ecs, typename Includes = TypeList<>, typename Excludes = TypeList<>>
        struct Group {};

        template<typename EcsType, typename... Includes, typename... Excludes>
        struct Group<EcsType, TypeList<Includes...>, TypeList<Excludes...>> : public GroupContainer {
          private:
            using Inc = TypeList<Includes...>;
            using Exc = TypeList<Excludes...>;
            friend class MGMecs<EntityType>;
            static constexpr auto const_group = std::is_const<EcsType>{};
            using G = Group<EcsType, TypeList<Includes...>, TypeList<Excludes...>>;

            EcsType* ecs = nullptr;
            GroupCond cond{};

            virtual void ecs_moved(const Ecs* new_loc) override {
                ecs = const_cast<EcsType*>(new_loc);
            }

          public:
            Group(const std::nullptr_t) {}

            Group(EcsType& from_ecs, const GroupCond& condition = {})
                : ecs(&from_ecs),
                  cond(condition) {
                ecs->groups.insert(reinterpret_cast<GroupContainer*>(this));
            }

            Group(const Group& g)
                : ecs(g.ecs),
                  cond(g.cond) {
                if (g.ecs == nullptr)
                    return;
                std::unique_lock lock{g.ecs->mutex};

                ecs->groups.insert(reinterpret_cast<GroupContainer*>(this));
            }
            Group(Group&& g)
                : ecs(std::move(g.ecs)),
                  cond(std::move(g.cond)) {
                if (g.ecs == nullptr)
                    return;
                std::unique_lock lock{g.ecs->mutex};

                ecs->groups.erase(reinterpret_cast<GroupContainer*>(&g));
                ecs->groups.emplace(reinterpret_cast<GroupContainer*>(this));
            }

            Group& operator=(const Group& g) {
                if (this == &g)
                    return *this;

                if (ecs == g.ecs) {
                    if (ecs == nullptr)
                        return *this;

                    std::unique_lock lock(ecs->mutex);
                    ecs->groups.emplace(reinterpret_cast<GroupContainer*>(this));
                }
                else {
                    if (ecs != nullptr) {
                        std::unique_lock lock{ecs->mutex};
                        ecs->groups.erase(reinterpret_cast<GroupContainer*>(this));
                    }
                    if (g.ecs != nullptr) {
                        std::unique_lock lock{g.ecs->mutex};
                        g.ecs->groups.emplace(reinterpret_cast<GroupContainer*>(this));
                    }

                    ecs = g.ecs;
                    cond = g.cond;
                }

                return *this;
            }
            Group& operator=(Group&& g) {
                if (this == &g)
                    return *this;

                if (ecs == g.ecs) {
                    if (ecs == nullptr)
                        return *this;

                    std::unique_lock lock(ecs->mutex);
                    ecs->groups.erase(reinterpret_cast<GroupContainer*>(&g));
                    ecs->groups.emplace(reinterpret_cast<GroupContainer*>(this));
                }
                else {
                    if (ecs != nullptr) {
                        std::unique_lock lock{ecs->mutex};
                        ecs->groups.erase(reinterpret_cast<GroupContainer*>(this));
                    }
                    if (g.ecs != nullptr) {
                        std::unique_lock lock{g.ecs->mutex};
                        g.ecs->groups.erase(reinterpret_cast<GroupContainer*>(&g));
                        g.ecs->groups.emplace(reinterpret_cast<GroupContainer*>(this));
                    }

                    ecs = g.ecs;
                    cond = g.cond;
                }

                g.ecs = nullptr;
                g.cond = {};
                return *this;
            }

            template<typename... Ts>
            auto include() {
                using Group2 = Group<EcsType, TypeList<Includes..., Ts...>, TypeList<Excludes...>>;
                if (ecs == nullptr)
                    return Group2{nullptr};
                return Group2{*ecs, cond};
            }
            template<typename... Ts>
            auto exclude() {
                using Group2 = Group<EcsType, TypeList<Includes...>, TypeList<Excludes..., Ts...>>;
                if (ecs == nullptr)
                    return Group2{nullptr};
                return Group2{*ecs, cond};
            }

            template<bool reverse = false>
            struct Iterator {
                using iterator_category = std::bidirectional_iterator_tag;
                using value_type = Entity;
                using difference_type = size_t;
                using pointer = Entity*;
                using reference = Entity&;

                using Bucket = ComponentBucket<typename Inc::First>;

                template<typename... Ts>
                struct EntityDeref {
                    using Map = MGMecsConstexprMap<size_t, size_t>;
                    using void_t = std::conditional_t<std::is_const_v<EcsType>, const void, void>;

                    EntityDeref() = default;
                    EntityDeref(const EntityDeref&) = default;
                    EntityDeref(EntityDeref&&) = default;
                    EntityDeref& operator=(const EntityDeref&) = default;
                    EntityDeref& operator=(EntityDeref&&) = default;

                    template<size_t... Is>
                    static constexpr Map make_map(std::index_sequence<Is...>) {
                        return Map{
                            Map::Item{TypeID<Ts>{}, Is}
                            ...
                        };
                    }

                    static constexpr Map map = make_map(std::index_sequence_for<Ts...>{});

                    void_t* ptrs[sizeof...(Ts)]{};
                    Entity e{};

                    operator Entity() const {
                        return e;
                    }

                    template<typename T, std::enable_if_t<!std::is_const_v<T>, bool> = true>
                    auto& get() const {
                        using T_t = std::conditional_t<std::is_const_v<EcsType>, const T, T>;
                        return *reinterpret_cast<T_t*>(ptrs[map[TypeID<T>{}]]);
                    }
                };

                EntityLock entity_lock{};
                const G* group = nullptr;
                size_t p = 0;
                EntityDeref<Includes...> deref_this{};
                std::unordered_set<size_t> unreachable{};

                Bucket::CompMoveCallbacks::CallbackHandle last_move_callback{};

              private:
                template<typename T = typename Inc::First>
                Bucket& get_bucket() const {
                    return group->ecs->template get_bucket<T>();
                }

                void setup_callback(Bucket& bucket) {
                    last_move_callback = bucket.comp_move_callbacks.create(bucket.original.back(), [this](Bucket& originating_bucket, size_t from, size_t to) {
                        if (from == p)
                            throw std::runtime_error("INTERNAL ERROR: Iterator callback was not listening to the last component in the bucket");
                        if (to == p)
                            throw std::runtime_error("An entity was destroyed while at least one iterator was pointing to it");

                        if (from > p && to < p)
                            unreachable.emplace(to);

                        setup_callback(originating_bucket);
                    });
                }

              public:
                Iterator(const Iterator&) = delete;
                Iterator(Iterator&& other)
                    : entity_lock(std::move(other.entity_lock)),
                      group(other.group),
                      p(other.p),
                      deref_this(other.deref_this) {
                    other.group = nullptr;
                    other.p = static_cast<size_t>(-1);
                    other.deref_this = {};

                    Bucket& bucket = get_bucket();
                    std::unique_lock lock{bucket.mutex};
                    setup_callback(bucket);
                }
                Iterator& operator=(const Iterator&) = delete;
                Iterator& operator=(Iterator&& other) {
                    if (this == &other)
                        return *this;

                    entity_lock = std::move(other.entity_lock);
                    group = other.group;
                    p = other.p;
                    deref_this = other.deref_this;
                    other.group = nullptr;
                    other.p = static_cast<size_t>(-1);
                    other.deref_this = {};

                    Bucket& bucket = get_bucket();
                    std::unique_lock lock{bucket.mutex};
                    setup_callback(bucket);

                    return *this;
                }

              private:
                template<typename T>
                bool setup_single_comp_in_entity_deref(const Entity e, Bucket& existing_bucket) {
                    if constexpr (std::is_same_v<ComponentBucket<T>, Bucket>)
                        return (deref_this.ptrs[deref_this.map[TypeID<T>{}]] = reinterpret_cast<EntityDeref<Includes...>::void_t*>(existing_bucket.try_get(e))) != nullptr;
                    else
                        return (deref_this.ptrs[deref_this.map[TypeID<T>{}]] = reinterpret_cast<EntityDeref<Includes...>::void_t*>(get_bucket<T>().try_get(e))) != nullptr;
                }
                template<typename T>
                bool contains(const Entity e, Bucket& existing_bucket) const {
                    if constexpr (std::is_same_v<ComponentBucket<T>, Bucket>)
                        return existing_bucket.try_get(e) != nullptr;
                    else
                        return get_bucket<T>().try_get(e) != nullptr;
                }

                bool setup_entity_deref(size_t c, Bucket& bucket) {
                    if (c >= bucket.original.size()) {
                        deref_this = {};
                        return false;
                    }
                    const auto e = bucket.original[c];
                    const bool success = (setup_single_comp_in_entity_deref<Includes>(e, bucket) || ...) && !(contains<Excludes>(e, bucket) || ...);
                    if (success)
                        deref_this.e = e;
                    else
                        deref_this = {};
                    return success;
                }

                // Needed because trying to lock the entity while the bucket lock is still locked can cause a deadlock if another thread is waiting for the bucket before letting go of the entity
                // But unlocking the bucket might give enough time for the entity to be destroyed or moved before it can be locked
                void setup_lock(std::unique_lock<std::recursive_mutex>& bucket_lock) {
                    entity_lock.invalidate();
                    std::unique_lock locks_lock{group->ecs->locks.mutex};
                    bucket_lock.unlock();

                    if (!group->ecs->locks.allow_locking || group->ecs->locks.map.find(deref_this.e) != group->ecs->locks.map.end())
                        group->ecs->locks.cv.wait(locks_lock, [this] {
                            if (!group->ecs->locks.allow_locking)
                                return false;

                            const auto it = group->ecs->locks.map.find(deref_this.e);
                            if (it == group->ecs->locks.map.end())
                                return true;
                            return it->second.thread_id == std::this_thread::get_id();
                        });
                    group->ecs->locks._wait_and_lock(deref_this.e);

                    entity_lock.e = deref_this.e;
                    entity_lock.is_locked = true;
                    entity_lock.ecs = group->ecs;

                    bucket_lock.lock();

                    locks_lock.unlock();
                    group->ecs->locks.cv.notify_all();
                }

              public:
                Iterator(const G* owner_group, size_t start_component)
                    : group(owner_group),
                      p(start_component) {
                    if (group == nullptr || p == static_cast<size_t>(-1))
                        return;

                    Bucket& bucket = get_bucket();
                    std::unique_lock lock{bucket.mutex};

                    while (!setup_entity_deref(p, bucket)) {
                        ++p;
                        if (p >= bucket.components.size()) {
                            p = static_cast<size_t>(-1);
                            group = nullptr;
                            entity_lock.invalidate();
                            deref_this = {};
                            return;
                        }
                    }

                    setup_lock(lock);
                    setup_callback(bucket);
                }

                bool is_end() const {
                    return group != nullptr && p == static_cast<size_t>(-1);
                }

                void inc() {
                    if (group == nullptr)
                        return;

                    Bucket& bucket = get_bucket();
                    std::unique_lock lock{bucket.mutex};

                    do {
                        if (!unreachable.empty()) {
                            const auto it = unreachable.begin();
                            p = *it;
                            unreachable.erase(it);
                        }
                        else if (p < bucket.components.size() - 1)
                            ++p;
                        else {
                            p = static_cast<size_t>(-1);
                            entity_lock.invalidate();
                            deref_this = {};
                            return;
                        }
                    }
                    while (!setup_entity_deref(p, bucket));

                    setup_lock(lock);
                }
                void dec() {
                    if (group == nullptr)
                        return;

                    Bucket& bucket = get_bucket();
                    std::unique_lock lock{bucket.mutex};

                    do {
                        --p;

                        if (p != static_cast<size_t>(-1) && !unreachable.empty()) {
                            auto it = unreachable.find(p);
                            while (it != unreachable.end()) {
                                unreachable.erase(it);
                                --p;
                                if (p == static_cast<size_t>(-1))
                                    break;
                                it = unreachable.find(p);
                            }
                        }

                        if (p == static_cast<size_t>(-1)) {
                            entity_lock.invalidate();
                            deref_this = {};
                            return;
                        }
                    }
                    while (!setup_entity_deref(p, bucket));

                    setup_lock(lock);
                }

                template<bool R = reverse, std::enable_if_t<!R, bool> = true>
                Iterator& operator++() {
                    inc();
                    return *this;
                }
                template<bool R = reverse, std::enable_if_t<R, bool> = true>
                Iterator& operator++() {
                    dec();
                    return *this;
                }
                Iterator& operator++(int) {
                    Iterator cpy{*this};
                    operator++();
                    return cpy;
                }

                template<bool R = reverse, std::enable_if_t<!R, bool> = true>
                Iterator& operator--() {
                    dec();
                    return *this;
                }
                template<bool R = reverse, std::enable_if_t<R, bool> = true>
                Iterator& operator--() {
                    inc();
                    return *this;
                }
                Iterator& operator--(int) {
                    Iterator cpy{*this};
                    operator--();
                    return cpy;
                }

                bool operator==(const Iterator& other) const {
                    return group == other.group && p == other.p && unreachable.empty();
                }
                bool operator!=(const Iterator& other) const { return !(*this == other); }

                difference_type operator-(const Iterator& other) const {
                    if (group == nullptr || other.group == nullptr)
                        return 0;
                    if (other.p >= p)
                        return 0;
                    if (p == static_cast<size_t>(-1)) {
                        Bucket& bucket = get_bucket();
                        std::unique_lock lock{bucket.mutex};
                        return bucket.original.size() - other.p;
                    }
                    return p - other.p;
                }

                EntityDeref<Includes...> operator*() const {
                    return deref_this;
                }
                EntityDeref<Includes...> operator->() const {
                    return **this;
                }
            };

            Iterator<> begin() const {
                if (ecs == nullptr)
                    return Iterator{this, static_cast<size_t>(-1)};

                return Iterator{this, 0};
            }
            Iterator<> end() const {
                return Iterator{this, static_cast<size_t>(-1)};
            }

            void invalidate_self() {
                if (ecs == nullptr)
                    return;

                std::unique_lock lock{ecs->mutex};

                ecs->groups.erase(reinterpret_cast<GroupContainer*>(this));
                ecs = nullptr;
                cond = {};
            }

            ~Group() {
                invalidate_self();
            }
        };

        Group<Ecs, TypeList<>, TypeList<>> group(const GroupCond& condition = {}) {
            return Group<Ecs, TypeList<>, TypeList<>>{*this, condition};
        }

        Group<const Ecs, TypeList<>, TypeList<>> group(const GroupCond& condition = {}) const {
            return Group<const Ecs, TypeList<>, TypeList<>>{*this, condition};
        }

        void wait_and_lock(const Entity e) {
            locks.wait_and_lock(e);
        }
        template<typename It>
        void wait_and_lock(const It& start, const It& end) {
            locks.wait_and_lock(start, end);
        }

        void unlock(const Entity e) {
            locks.unlock(e);
        }
        template<typename It>
        void unlock(const It& start, const It& end) {
            locks.unlock(start, end);
        }

        ~MGMecs() {
            while (!buckets.empty()) {
                const auto it = buckets.begin();
                const auto bucket = it->second;

                while (bucket->count() != 0)
                    bucket->destroy_all(this);
                delete bucket;
                buckets.erase(it);
            }
        }
    };
} // namespace mgm
