#pragma once
#include <atomic>
#include <cstdint>
#include <iterator>
#include <memory.h>
#include <functional>
#include <type_traits>
#include <unordered_map>
#include <stdexcept>
#include <vector>


namespace mgm {
    using Entity = uint32_t;
    using Component = uint32_t;
    static constexpr Entity null = (Entity)-1;
    static constexpr Component null_component = (Component)-1;
    static constexpr uint32_t null_type_id = (uint32_t)-1;

    class MgmEcs;

    template<typename, typename = void>
    constexpr bool has_callback_emplace = false;
    template<typename T>
    constexpr bool has_callback_emplace<T, std::void_t<std::enable_if<std::is_same_v<void (T::*)(MgmEcs& ecs, Entity entity), decltype(&T::on_emplace)>>>> = true;

    template<typename, typename = void>
    constexpr bool has_callback_access = false;
    template<typename T>
    constexpr bool has_callback_access<T, std::void_t<std::enable_if<std::is_same_v<void (T::*)(MgmEcs& ecs, Entity entity), decltype(&T::on_access)>>>> = true;

    template<typename, typename = void>
    constexpr bool has_callback_remove = false;
    template<typename T>
    constexpr bool has_callback_remove<T, std::void_t<std::enable_if<std::is_same_v<void (T::*)(MgmEcs& ecs, Entity entity), decltype(&T::on_remove)>>>> = true;


    template<typename... Ts>
    struct TypeList {
        static constexpr uint32_t num_types = sizeof...(Ts);

        template<uint32_t i, std::enable_if_t<(i < num_types), bool> = true>
        static constexpr auto get_at() {
            if constexpr (i == 0)
                return _First<TypeList<Ts...>>{};
            else
                return typename _First<TypeList<Ts...>>::Remaining{}.template get_at<i - 1>();
        }

        template<uint32_t i, std::enable_if_t<(i < num_types), bool> = true>
        static constexpr auto get_until() {
            if constexpr (i == 0)
                return TypeList<typename _First<TypeList<Ts...>>::Type>{};
            else
                return typename decltype(get_until<i - 1>())::template Append<TypeAt<i>>{};
        }

        template<uint32_t i, std::enable_if_t<(i < num_types), bool> = true>
        static constexpr auto get_after_including_current() {
            if constexpr (i == num_types - 1)
                return TypeList<TypeAt<i>>{};
            else
                return typename decltype(get_after_including_current<i + 1>())::template Prepend<TypeAt<i>>{};
        }

        template<uint32_t i = num_types - 1>
        static constexpr size_t size() {
            if constexpr (sizeof...(Ts) == 0)
                return 0;
            else if constexpr (i == 0)
                return sizeof(typename _First<TypeList<Ts...>>::Type);
            else
                return sizeof(typename _First<TypeList<Ts...>>::Type) + typename _First<TypeList<Ts...>>::Remaining{}.template size<i - 1>();
        }

        template<typename T>
        struct _First;
        template<typename T, typename... Us>
        struct _First<TypeList<T, Us...>> {
            using Type = T;
            using Remaining = TypeList<Us...>;
        };

        template<uint32_t i>
        using TypeAt = decltype(get_at<i>())::Type;

        template<typename T>
        using Append = TypeList<Ts..., T>;

        template<typename T>
        using Prepend = TypeList<T, Ts...>;

        template<typename T>
        struct _Concat;
        template<typename... Us>
        struct _Concat<TypeList<Us...>> {
            using Result = TypeList<Ts..., Us...>;
        };
        template<typename T>
        using Concat = _Concat<T>::Result;

        template<uint32_t i>
        using TypesUntil = decltype(get_until<i>());

        template<uint32_t i>
        using TypesAfter = decltype(get_after_including_current<i + 1>());
    };

    template<typename T>
    struct TypeListContainer;

    template<typename... Ts>
    struct TypeListContainer<TypeList<Ts...>> {
        uint8_t data[TypeList<Ts...>::size()]{};

        private:
        template<uint32_t i = 0>
        void initialize() {
            using T = TypeList<Ts...>::template TypeAt<i>;
            new (&get<i>()) T{};
            if constexpr (i < TypeList<Ts...>::num_types - 1)
                initialize<i + 1>();
        }

        public:
        TypeListContainer() {
            initialize();
        }

        template<uint32_t i>
        auto& get() {
            using FirstList = decltype(TypeList<Ts...>{}.template get_at<i>());
            using First = typename FirstList::Type;
            return *reinterpret_cast<First*>(&data[TypeList<Ts...>{}.size() - typename FirstList::Remaining{}.size() - sizeof(First)]);
        }

        template<uint32_t i>
        const auto& get() const {
            return const_cast<TypeListContainer*>(this)->get<i>();
        }

        private:
        template<uint32_t i = 0>
        void destruct() {
            using T = TypeList<Ts...>::template TypeAt<i>;
            get<i>().~T();
            if constexpr (i < TypeList<Ts...>::num_types - 1)
                destruct<i + 1>();
        }

        public:
        ~TypeListContainer() {
            destruct();
        }
    };


    class MgmEcs {
        friend class EntityReference;
        friend class ComponentReference;

        static inline std::atomic_uint32_t _num_types = 0;
        public:
        /**
         * @brief Get the ID of the given type
         * 
         * @tparam T The type to get the ID of
         */
        template<typename T> static inline const uint32_t type_id = _num_types++;


        struct EntityReference {
            private:
            friend class MgmEcs;
            EntityReference(const MgmEcs& ecs, const Entity entity) : num_refs{new uint32_t{1}}, ecs{&ecs}, entity{entity} {}

            public:
            EntityReference() = default;

            EntityReference(const EntityReference& cr) : num_refs{cr.num_refs}, ecs{cr.ecs}, entity{cr.entity} {
                ++*num_refs;
            }
            EntityReference(EntityReference&& cr) : num_refs{cr.num_refs}, ecs{cr.ecs}, entity{cr.entity} {
                memset(&cr, 0, sizeof(EntityReference));
            }
            EntityReference& operator=(const EntityReference& cr) {
                if (this == &cr)
                    return *this;
                memcpy(this, &cr, sizeof(EntityReference));
                ++*num_refs;
                return *this;
            }
            EntityReference& operator=(EntityReference&& cr) {
                if (this == &cr)
                    return *this;
                memcpy(this, &cr, sizeof(EntityReference));
                memset(&cr, 0, sizeof(EntityReference));
                return *this;
            }

            uint32_t* num_refs = nullptr;
            const MgmEcs* ecs = nullptr;
            Entity entity = null;

            bool is_valid() const { return ecs != nullptr; }
            template<typename T> const T& get() const {
                if (ecs == nullptr)
                    throw std::runtime_error("Invalid reference, or wrong type");
                const auto& map = ecs->get_type_pool_map<T>();
                return map.get(entity);
            }
            void destroy_original() {
                if (ecs == nullptr)
                    throw std::runtime_error("Invalid reference");
                if (*num_refs > 1)
                    throw std::runtime_error("Other references are using this entity, so it cannot be destroyed");
                const_cast<MgmEcs*>(ecs)->destroy(entity);
                invalidate();
            }

            void invalidate() {
                if (num_refs == nullptr)
                    return;

                --*num_refs;
                if (*num_refs == 0)
                    delete num_refs;

                num_refs = nullptr;
                ecs = nullptr;
                entity = null;
            }

            ~EntityReference() {
                invalidate();
            }
        };
        struct ComponentReference : public EntityReference {
            private:
            friend class MgmEcs;
            using EntityReference::EntityReference;

            ComponentReference(const MgmEcs& ecs, const Entity entity, const uint32_t type_id) {
                this->num_refs = new uint32_t{1};
                this->ecs = &ecs;
                this->entity = entity;
                this->type_id = type_id;
            }

            public:
            uint32_t type_id = null;

            template<typename T> const T& get() const {
                if (this->ecs == nullptr || type_id != MgmEcs::type_id<T>)
                    throw std::runtime_error("Invalid reference, or wrong type");
                const auto& map = this->ecs->template get_type_pool_map<T>();
                return map.get(this->entity);
            }
            void destroy_original() {
                if (this->ecs == nullptr)
                    throw std::runtime_error("Invalid reference");
                if (*this->num_refs > 1)
                    throw std::runtime_error("Other references are using this component, so it cannot be destroyed");
                MgmEcs::ComponentPool& type_pool = const_cast<MgmEcs*>(this->ecs)->type_pools[type_id];
                type_pool.remove(*const_cast<MgmEcs*>(ecs), this->entity);
                this->invalidate();
            }

            ~ComponentReference() {
                this->invalidate();
            }
        };


        private:
        template<typename Type, size_t S = 256>
        class PagedBinarySearchMap {
            public:
            struct Iterator {
                friend class PagedBinarySearchMap<Type>;
                const PagedBinarySearchMap<Type>* map = nullptr;
                uint32_t page = 0;
                uint32_t pos = 0;

                Iterator() = delete;

                private:
                Iterator(const PagedBinarySearchMap<Type>& map, const uint32_t page, const uint32_t pos) : map{&map}, page{page}, pos{pos} {}

                public:
                Iterator operator++() {
                    if (page == map->pages.size())
                        return *this;
                    ++pos;
                    if (pos == map->pages[page].num) {
                        ++page;
                        pos = 0;
                    }
                    return *this;
                }
                bool operator==(const Iterator& it) const { return map == it.map && page == it.page && pos == it.pos; }
                bool operator!=(const Iterator& it) const { return !(*this == it); }
                Entity operator*() const {
                    if (page == map->pages.size())
                        return null;
                    return map->pages[page].data[pos].entity;
                }
            };

            private:
            struct PageMember {
                Type value;
                Entity entity = null;
            };
            struct Page {
                Component* map = nullptr;
                PageMember* data = nullptr;
                Component num = 0;
                uint32_t id = 0;
            };
            std::vector<Page> pages{};
            uint32_t cached_page_id = 0;
            uint32_t cached_page_index = 0;

            struct PageIndex {
                uint32_t pos : (sizeof(uint32_t) * 8 - 1) = 0;
                bool is_insert_point : 1 = false;
            };
            PageIndex try_get_page_index(const uint32_t page_id) const {
                uint32_t i = pages.size() / 2;
                uint32_t left = 0, right = pages.size();

                do {
                    if (pages[i].id > page_id) {
                        right = i;
                        i = (left + i) / 2;
                    }
                    else if (pages[i].id < page_id) {
                        left = i;
                        i = (i + right + 1) / 2;
                    }
                    else if (pages[i].id == page_id)
                        return PageIndex{ i, false };
                } while (right - left > 1);
                return PageIndex{ left, pages[left].id != page_id };
            }
            Page& get_page(const uint32_t page_id) const {
                if (page_id == cached_page_id) return const_cast<Page&>(pages[cached_page_index]);

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point) {
                    const_cast<PagedBinarySearchMap<Type, S>*>(this)->cached_page_id = page_id;
                    const_cast<PagedBinarySearchMap<Type, S>*>(this)->cached_page_index = page.pos;
                    return const_cast<Page&>(pages[page.pos]);
                }

                throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
            }
            Page& get_or_create_page(const uint32_t page_id) {
                if (page_id == cached_page_id) return pages[cached_page_index];

                PageIndex page = try_get_page_index(page_id);
                cached_page_id = page_id;
                if (!page.is_insert_point) {
                    cached_page_index = page.pos;
                    return pages[page.pos];
                }

                page.pos++;
                const auto& it = pages.emplace(pages.begin() + page.pos, Page{new Component[S], std::allocator<PageMember>{}.allocate(S), 0, page_id});
                for (uint32_t i = 0; i < S; i++) {
                    it->data[i].entity = null;
                    it->map[i] = null_component;
                }
                cached_page_index = page.pos;
                return pages[page.pos];
            }

            public:
            PagedBinarySearchMap(const PagedBinarySearchMap<Type, S>&) = delete;
            PagedBinarySearchMap(PagedBinarySearchMap<Type, S>&& pm) {
                if (this == &pm)
                    return;
                pages = std::move(pm.pages);
            }
            PagedBinarySearchMap<Type, S>& operator=(const PagedBinarySearchMap<Type, S>&) = delete;
            PagedBinarySearchMap<Type, S>& operator=(PagedBinarySearchMap<Type, S>&& pm) {
                if (this == &pm)
                    return *this;
                pages = std::move(pm.pages);
                return *this;
            }

            PagedBinarySearchMap() {
                const Page& page = pages.emplace_back(Page{new Component[S], std::allocator<PageMember>{}.allocate(S), 0, 0});
                for (uint32_t i = 0; i < S; i++) {
                    page.data[i].entity = null;
                    page.map[i] = null_component;
                }
            }

            template<typename... Ts>
            Type& emplace(const Entity entity, Ts&&... args) {
                Page& page = get_or_create_page(entity / S);
                Component& component = page.map[entity % S];
                if (component != null_component)
                    throw std::runtime_error("Entity already exists");
                component = page.num++;
                new (&page.data[component].value) Type{args...};
                page.data[component].entity = entity;
                return page.data[component].value;
            }

            template<typename... Ts>
            Type& get_or_emplace(const Entity entity, Ts&&... args) {
                Page& page = get_or_create_page(entity / S);
                Component& component = page.map[entity % S];
                if (component == null_component) {
                    component = page.num++;
                    new (&page.data[component].value) Type{args...};
                    page.data[component].entity = entity;
                }
                return page.data[component].value;
            }

            Type& get(const Entity entity) const {
                const Page& page = get_page(entity / S);
                const Component& component = page.map[entity % S];
                if (component == null_component)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                return *const_cast<Type*>(&page.data[component].value);
            }

            Type* try_get(const Entity entity) const {
                const PageIndex page_index = try_get_page_index(entity / S);
                if (page_index.is_insert_point)
                    return nullptr;
                const Page& page = pages[page_index.pos];
                const Component& component = page.map[entity % S];
                if (component == null_component)
                    return nullptr;
                return const_cast<Type*>(&page.data[component].value);
            }

            bool has(const Entity entity) const {
                const PageIndex page_index = try_get_page_index(entity / S);
                if (page_index.is_insert_point)
                    return false;
                return pages[page_index.pos].map[entity % S] != null_component;
            }

            void destroy(const Entity entity) {
                Page& page = get_page(entity / S);
                Component& component = page.map[entity % S];
                if (component == null_component)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                std::swap(page.data[component], page.data[--page.num]);
                page.map[page.data[component].entity % S] = component;
                page.data[page.num].value.~Type();
                page.data[page.num].entity = null;
                component = null_component;
            }
            void try_destroy(Entity entity) {
                const PageIndex& page_index = try_get_page_index(entity / S);
                if (page_index.is_insert_point)
                    return;
                Page& page = pages[page_index.pos];
                Component& component = page.map[entity % S];
                if (component == null_component)
                    return;
                std::swap(page.data[component], page.data[--page.num]);
                page.map[page.data[component].entity % S] = component;
                page.data[page.num].value.~Type();
                page.data[page.num].entity = null;
                component = null_component;
            }

            Iterator begin() const { return Iterator{*this, 0, 0}; }
            Iterator end() const { return Iterator{*this, (uint32_t)(pages.size()), 0 }; }

            ~PagedBinarySearchMap() {
                for (auto& page : pages) {
                    for (uint32_t i = 0; i < page.num; i++) {
                        if (page.data[i].entity != null) {
                            page.data[i].value.~Type();
                            memset(&page.data[i].value, 0, sizeof(S));
                        }
                    }
                    std::allocator<PageMember>{}.deallocate(page.data, S);
                }
            }
        };


        enum EntityFlags {
            EntityFlag_NONE = 0x0,
            EntityFlag_TOOMBSTONE = 0x1 << 0
        };
        struct EntityInfo {
            EntityFlags flags = EntityFlag_NONE;
        };
        struct ComponentPool {
            void* data = nullptr;

            std::function<void(MgmEcs&, const Entity)> remove{};
            std::function<void(MgmEcs&, const Entity)> try_remove{};
            std::function<bool(const MgmEcs&, const Entity)> has{};
            std::function<void(ComponentPool& pool)> free{};

            template<typename T>
            PagedBinarySearchMap<T>& map() const {
                return *reinterpret_cast<PagedBinarySearchMap<T>*>(data);
            }

            ~ComponentPool() {
                free(*this);
            }
        };
        template<typename T>
        PagedBinarySearchMap<T>& get_or_create_type_pool_map() {
            auto it = type_pools.find(type_id<T>);
            if (it != type_pools.end())
                return it->second.template map<T>();

            ComponentPool& pool = create_type_pool<T>();
            return pool.template map<T>();
        }
        template<typename T>
        PagedBinarySearchMap<T>& get_type_pool_map() const {
            auto it = type_pools.find(type_id<T>);
            if (it == type_pools.end())
                throw std::runtime_error("Tried to access unregistered type");
            const ComponentPool& pool = it->second;
            return const_cast<PagedBinarySearchMap<T>&>(pool.template map<T>());
        }
        std::unordered_map<uint32_t, ComponentPool> type_pools{};

        std::vector<EntityInfo> entities{};
        std::vector<Entity> available_entities{};

        template<typename T>
        ComponentPool& create_type_pool() {
            ComponentPool& pool = type_pools[type_id<T>];

            pool.data = new PagedBinarySearchMap<T>;
            pool.remove = [](MgmEcs& ecs, const Entity entity) { ecs.remove<T>(entity); };
            pool.try_remove = [](MgmEcs& ecs, const Entity entity) { ecs.try_remove<T>(entity); };
            pool.has = [](const MgmEcs& ecs, const Entity entity) { return ecs.try_get<T>(entity) != nullptr; };
            pool.free = [](ComponentPool& pool) { delete static_cast<PagedBinarySearchMap<T>*>(pool.data); };
            return pool;
        }

        public:
        MgmEcs(const MgmEcs&) = delete;
        MgmEcs(MgmEcs&&) = delete;
        MgmEcs& operator=(const MgmEcs&) = delete;
        MgmEcs& operator=(MgmEcs&&) = delete;

        MgmEcs() = default;

        /**
         * @brief Create an entity
         * 
         * @return The ID of the entity
         */
        Entity create() {
            if (available_entities.empty()) {
                EntityInfo& new_entity = entities.emplace_back();
                return entities.size() - 1;
            }
            const auto entity = available_entities.back();
            available_entities.pop_back();
            entities[entity] = EntityInfo{};
            return entity;
        }

        bool is_toombstone(const Entity entity) {
            return (bool)(entities[entity].flags & EntityFlag_TOOMBSTONE);
        }

        /**
         * @brief Create a number of entities
         * 
         * @param num How many entities to create
         * @return A vector with the IDs of the entities
         */
        template<typename It,
            std::iterator_traits<It>::iterator_category = typename std::iterator_traits<It>::iterator_category{}>
        void create(const It& begin, const It& end) {
            const auto num_create = end - begin;
            if (available_entities.empty()) {
                auto num_entities = entities.size();
                entities.resize(num_entities + num_create);
                for (auto e = begin; e != end; e++)
                    *e = num_entities++;
            }
            else if (available_entities.size() >= num_create) {
                for (auto e = begin; e != end; e++) {
                    *e = available_entities.back();
                    available_entities.pop_back();
                    entities[*e] = EntityInfo{};
                }
            }
            else {
                for (auto e = begin; e != end; e++) {
                    *e = available_entities.back();
                    available_entities.pop_back();
                    entities[*e] = EntityInfo{};
                    if (available_entities.empty()) {
                        create(e + 1, end);
                        break;
                    }
                }
            }
        }

        /**
         * @brief Get a protective reference to an entity
         * 
         * @param entity The ID of the entity to get
         * @return The reference to the entity
         */
        EntityReference entity_reference(const Entity entity) const {
            return EntityReference{*this, entity};
        }

        /**
         * @brief Destroy an entity
         * 
         * @param entity The ID of the entity to destroy
         */
        void destroy(const Entity entity) {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                throw std::runtime_error("Entity doesn't exist");

            auto components = get_all(entity);
            for (auto& type_pool : type_pools)
                type_pool.second.try_remove(*this, entity);
            entities[entity].flags = EntityFlag_TOOMBSTONE;
            available_entities.push_back(entity);
        }

        /**
         * @brief Destroy a number of entities
         * 
         * @param begin An iterator to the beginning of the list of entities
         * @param end An iterator to the end of the list of entities
         */
        template<typename It,
            std::iterator_traits<It>::iterator_category = typename std::iterator_traits<It>::iterator_category{}>
        void destroy(const It& begin, const It& end) {
            for (auto e = begin; e != end; e++) {
                for (auto& [tid, type_pool] : type_pools)
                    type_pool.try_remove(*this, *e);
                this->entities[*e].flags = EntityFlag_TOOMBSTONE;
                available_entities.push_back(*e);
            }
        }

        template<typename T, typename U>
        struct Group;

        template<typename... Ts, typename... Us>
        struct Group<TypeList<Ts...>, TypeList<Us...>> {
            friend class MgmEcs;
            using IncludeTypes = TypeList<Ts...>;
            using ExcludeTypes = TypeList<Us...>;
            using MainType = IncludeTypes::template TypeAt<0>;

            struct Iterator : std::iterator_traits<Entity> {
                friend class Group<TypeList<Ts...>, TypeList<Us...>>;

                const MgmEcs* ecs;
                PagedBinarySearchMap<MainType>::Iterator current;

                Iterator() = default;

                private:
                Iterator(const MgmEcs& ecs, const PagedBinarySearchMap<MainType>::Iterator& it) : ecs{&ecs}, current{it} {}

                template<typename T>
                bool _valid() const {
                    if constexpr (T::num_types > 1) {
                        if (ecs->has<typename T::template TypeAt<0>>(*current))
                            return _valid<T::template TypesAfter<0>>();
                        else
                            return false;
                    }
                    else
                        return ecs->has<typename T::template TypeAt<0>>(*current);
                }

                public:
                bool valid() const {
                    if constexpr (IncludeTypes::num_types > 1) {
                        if constexpr(ExcludeTypes::num_types)
                            return _valid<typename IncludeTypes::template TypesAfter<0>>() && !_valid<TypeList<Us...>>();
                        else
                            return _valid<typename IncludeTypes::template TypesAfter<0>>();
                    }
                    else {
                        if constexpr(ExcludeTypes::num_types)
                            return !_valid<TypeList<Us...>>();
                        else
                            return true;
                    }
                }
                Iterator operator++() {
                    do
                        ++current;
                    while (!valid() && *current != null);
                    return *this;
                }
                bool operator==(const Iterator& it) const { return ecs == it.ecs && current == it.current; }
                bool operator!=(const Iterator& it) const { return !(*this == it); }
                Entity operator*() const { return *current; }
            };

            const MgmEcs* ecs;

            private:
            Iterator _begin;
            Iterator _end;

            Group(const MgmEcs& ecs) : ecs{&ecs}, _begin{ecs, ecs.get_type_pool_map<MainType>().begin()}, _end{ecs, ecs.get_type_pool_map<MainType>().end()} {}

            public:
            Group() = delete;

            auto begin() const { return _begin; }
            auto end() const { return _end; }

            template<typename T>
            Group(const PagedBinarySearchMap<T>::Iterator& begin, const PagedBinarySearchMap<T>::Iterator& end) {};
        };

        template<typename T, typename... Ts, typename... Us>
        auto group(const TypeList<T, Ts...>& include = TypeList<T, Ts...>{}, const TypeList<Us...>& exclude = TypeList<Us...>{}) const {
            return Group<TypeList<T, Ts...>, TypeList<Us...>>{*this};
        }

        /**
         * @brief Register a component for an entity, and call a constructor with the given arguments
         * 
         * @tparam T The type of component to create
         * @param entity The ID of the entity to create the component for
         * @param args Arguments to give to the constructor (can be none)
         * @return A reference to the component
         */
        template<typename T, typename... Ts>
        T& emplace(const Entity entity, Ts&&... args) {
            auto& res = get_or_create_type_pool_map<T>().emplace(entity, args...);
            if constexpr (has_callback_emplace<T>)
                res.on_emplace(*this, entity);
            return res;
        }

        /**
         * @brief Register a component for each entity in a list of entities, and call a constructor with the given arguments
         * 
         * @param begin An iterator to the beginning of the list of entities
         * @param end An iterator to the end of the list of entities
         * @param args Constructor arguments
         */
        template<typename T, typename It, typename... Ts,
            std::iterator_traits<It>::iterator_category = typename std::iterator_traits<It>::iterator_category{}>
        void emplace(const It& begin, const It& end, Ts&&... args) {
            auto& map = get_or_create_type_pool_map<T>();
            for (auto e = begin; e < end; e++) {
                if constexpr (has_callback_emplace<T>)
                    map.emplace(*e, args...).on_emplace(*this, *e);
                else
                    map.emplace(*e, args...);
            }
        }

        /**
         * @brief Get a reference to a component
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A reference to the component
         */
        template<typename T>
        T& get(const Entity entity) const {
            const PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            auto& res = map.get(entity);
            if constexpr (has_callback_access<T>)
                res.on_access(*this, entity);
            return res;
        }

        /**
         * @brief Get all components connected to an entity
         * 
         * @param entity The ID of the entity to get
         * @return A vector with Component References to all components
         */
        std::vector<ComponentReference> get_all(const Entity entity) const {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                throw std::runtime_error("Entity doesn't exist");

            std::vector<ComponentReference> components{};
            for (const auto& [type_id, type_pool] : type_pools) {
                if (!type_pool.has(*this, entity))
                    continue;
                ComponentReference cr{*this, entity, type_id};
                components.emplace_back(std::move(cr));
            }
            return components;
        }

        /**
         * @brief Get a component from an entity, emplacing it if doesn't already exist
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @param args Arguments to give to the constructor (can be none)
         * @return A reference to the component
         */
        template<typename T, typename... Ts>
        T& get_or_emplace(const Entity entity, Ts&&... args) {
            if constexpr (has_callback_emplace<T> || has_callback_access<T>) {
                PagedBinarySearchMap<T>& map = get_or_create_type_pool_map<T>();
                T* res = map.try_get(entity);
                if (res != nullptr) {
                    if constexpr (has_callback_access<T>)
                        res->on_access(*this, entity);
                    return res;
                }
                res = &map.emplace(entity, args...);
                if constexpr (has_callback_emplace<T>)
                    res->on_emplace(*this, entity);
                return res;
            }
            else
                return get_or_create_type_pool_map<T>().get_or_emplace(entity, args...);
        }

        /**
         * @brief Get a component from an entity
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A pointer to the component, or nullptr if it doesn't exist
         */
        template<typename T>
        T* try_get(const Entity entity) const {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                return nullptr;

            const PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            T* res = const_cast<T*>(map.try_get(entity));
            if constexpr (has_callback_access<T>)
                res->on_access(*this, entity);
            return res;
        }
        
        template<typename T>
        bool has(const Entity entity) const {
            return get_type_pool_map<T>().has(entity);
        }

        /**
         * @brief Remove a component from an entity and call its destructor
         * 
         * @tparam T The type of component to remove
         * @param entity The ID of the entity to remove the component from
         */
        template<typename T>
        void remove(const Entity entity) {
            PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            if constexpr (has_callback_remove<T>)
                map.get(entity).on_remove(*this, entity);
            map.destroy(entity);
        }

        /**
         * @brief Remove a component from a number of entities
         * 
         * @tparam T The type of component to remove
         * @param begin An iterator to the beginning of the list of entities
         * @param end An iterator to the end of the list of entities
         */
        template<typename T, typename It,
            std::iterator_traits<It>::iterator_category = typename std::iterator_traits<It>::iterator_category{}>
        void remove(const It& begin, const It& end) {
            PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            for (auto e = begin; e != end; e++) {
                if constexpr (has_callback_remove<T>)
                    map.get(*e).on_remove(*this, *e);
                map.destroy(*e);
            }
        }

        /**
         * @brief Remove a component from an entity and call its destructor, if it exists
         * 
         * @tparam T The type of component to remove
         * @param entity The ID of the entity to remove the component from
         */
        template<typename T>
        void try_remove(const Entity entity) {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                return;

            PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            if constexpr (has_callback_remove<T>) {
                auto ent = map.try_get(entity);
                if (ent != nullptr) {
                    map.get(entity).on_remove(*this, entity);
                    map.destroy(entity);
                }
            }
            else
                map.try_destroy(entity);
        }

        /**
         * @brief Remove a component from a number of entities and call its destructor, if it exists on that entity
         * 
         * @tparam T The type of component to remove
         * @param begin An iterator to the beginning of the list of entities
         * @param end An iterator to the end of the list of entities
         */
        template<typename T, typename It,
            std::iterator_traits<It>::iterator_category = typename std::iterator_traits<It>::iterator_category{}>
        void try_remove(const It& begin, const It& end) {
            PagedBinarySearchMap<T>& map = get_type_pool_map<T>();
            for (auto e = begin; e != end; e++) {
                if (this->entities[*e].flags & EntityFlag_TOOMBSTONE)
                    continue;

                if constexpr (has_callback_remove<T>) {
                    auto ent = map.try_get(*e);
                    if (ent != nullptr) {
                        map.get(*e).on_remove(*this, *e);
                        map.destroy(*e);
                    }
                }
                else
                    map.try_destroy(*e);
                }
        }

        ~MgmEcs() {
        }
    };
}
