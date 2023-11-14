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
    using EntityType = uint32_t;
    using ComponentType = uint32_t;

    template<typename Entity = EntityType, typename Component = ComponentType,
        std::enable_if_t<std::is_integral_v<EntityType>, bool> = true,
        std::enable_if_t<std::is_integral_v<ComponentType>, bool> = true
    >
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

        static constexpr Entity null = (Entity)-1;
        static constexpr Component null_component = (Component)-1;
        static constexpr uint32_t null_type_id = (uint32_t)-1;


        struct EntityReference {
            private:
            friend class MgmEcs<Entity, Component>;
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
            friend class MgmEcs<Entity, Component>;
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
                type_pool.remove(type_pool, this->entity);
                this->invalidate();
            }

            ~ComponentReference() {
                this->invalidate();
            }
        };


        private:
        template<typename Type, size_t S = 256>
        class PackedMapWithSparseSearch {
            struct PageMember {
                Type value;
                Entity entity = null;
            };
            struct Page {
                PageMember* data = nullptr;
                uint32_t id = 0;
            };
            std::vector<Page> pages{};
            uint32_t cached_page_id = 0;
            uint32_t cached_page_index = 0;

            struct PageIndex {
                bool is_insert_point = false;
                uint32_t pos = 0;
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
                        return PageIndex{ false, i };
                } while (right - left > 1);
                return PageIndex{ pages[left].id != page_id, left };
            }
            Page& get_page(const uint32_t page_id) const {
                if (page_id == cached_page_id) return const_cast<Page&>(pages[cached_page_index]);

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point) {
                    const_cast<PackedMapWithSparseSearch<Type, S>*>(this)->cached_page_id = page_id;
                    const_cast<PackedMapWithSparseSearch<Type, S>*>(this)->cached_page_index = page.pos;
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
                const auto& it = pages.emplace(pages.begin() + page.pos, Page{std::allocator<PageMember>{}.allocate(S), page_id});
                for (uint32_t i = 0; i < S; i++)
                    it->data[i].entity = null;
                cached_page_index = page.pos;
                return pages[page.pos];
            }

            public:
            PackedMapWithSparseSearch(const PackedMapWithSparseSearch<Type, S>&) = delete;
            PackedMapWithSparseSearch(PackedMapWithSparseSearch<Type, S>&& pm) {
                if (this == &pm)
                    return;
                pages = std::move(pm.pages);
            }
            PackedMapWithSparseSearch<Type, S>& operator=(const PackedMapWithSparseSearch<Type, S>&) = delete;
            PackedMapWithSparseSearch<Type, S>& operator=(PackedMapWithSparseSearch<Type, S>&& pm) {
                if (this == &pm)
                    return *this;
                pages = std::move(pm.pages);
                return *this;
            }

            PackedMapWithSparseSearch() {
                const auto& page = pages.emplace_back(Page{std::allocator<PageMember>{}.allocate(S), 0});
                for (uint32_t i = 0; i < S; i++)
                    page.data[i].entity = null;
            }

            template<typename... Ts>
            Type& emplace(const Entity entity, Ts&&... args) {
                PageMember& component = get_or_create_page(entity / S).data[entity % S];
                if (component.entity != null)
                    throw std::runtime_error("Entity already exists");
                memset(&component, 0, sizeof(PageMember));
                new (&component.value) Type{args...};
                component.entity = entity;
                return component.value;
            }

            template<typename... Ts>
            Type& get_or_emplace(const Entity entity, Ts&&... args) {
                PageMember& component = get_or_create_page(entity / S).data[entity % S];
                if (component.entity != null)
                    return component.value;
                memset(&component, 0, sizeof(PageMember));
                new (&component.value) Type{args...};
                component.entity = entity;
                return component.value;
            }

            Type& get(const Entity entity) const {
                const PageMember& component = get_page(entity / S).data[entity % S];
                if (component.entity == null)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                return *const_cast<Type*>(&component.value);
            }

            Type* try_get(const Entity entity) const {
                const PageIndex page_index = try_get_page_index(entity / S);
                if (page_index.is_insert_point)
                    return nullptr;
                const PageMember& component = pages[page_index.pos].data[entity % S];
                if (component.entity == null)
                    return nullptr;
                return const_cast<Type*>(&component.value);
            }

            void destroy(const Entity entity) {
                PageMember& component = get_page(entity / S).data[entity % S];
                if (component.entity == null)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                component.value.~Type();
                component.entity = null;
            }
            void try_destroy(Entity entity) {
                PageMember& component = get_page(entity / S).data[entity % S];
                if (component.entity == null)
                    return;
                component.value.~Type();
                component.entity = null;
            }

            ~PackedMapWithSparseSearch() {
                for (auto& page : pages) {
                    for (uint32_t i = 0; i < S; i++) {
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

            std::function<void(ComponentPool&, const Entity)> remove{};
            std::function<void(ComponentPool&, const Entity)> try_remove{};
            std::function<bool(const ComponentPool&, const Entity)> has{};
            std::function<void(ComponentPool& pool)> free{};

            template<typename T>
            PackedMapWithSparseSearch<T>& map() const {
                return *reinterpret_cast<PackedMapWithSparseSearch<T>*>(data);
            }

            ~ComponentPool() {
                free(*this);
            }
        };
        template<typename T>
        PackedMapWithSparseSearch<T>& get_or_create_type_pool_map() {
            auto it = type_pools.find(type_id<T>);
            if (it != type_pools.end())
                return it->second.template map<T>();

            ComponentPool& pool = create_type_pool<T>();
            return pool.template map<T>();
        }
        template<typename T>
        PackedMapWithSparseSearch<T>& get_type_pool_map() const {
            auto it = type_pools.find(type_id<T>);
            if (it == type_pools.end())
                throw std::runtime_error("Tried to access unregistered type");
            const ComponentPool& pool = it->second;
            return const_cast<PackedMapWithSparseSearch<T>&>(pool.template map<T>());
        }
        std::unordered_map<uint32_t, ComponentPool> type_pools{};

        std::vector<EntityInfo> entities{};
        std::vector<Entity> available_entities{};

        template<typename T>
        ComponentPool& create_type_pool() {
            ComponentPool& pool = type_pools[type_id<T>];

            pool.data = new PackedMapWithSparseSearch<T>;
            pool.remove = [](ComponentPool& pool, const Entity entity) { pool.template map<T>().destroy(entity); };
            pool.try_remove = [](ComponentPool& pool, const Entity entity) { pool.template map<T>().try_destroy(entity); };
            pool.has = [](const ComponentPool& pool, const Entity entity) { return pool.template map<T>().try_get(entity) != nullptr; };
            pool.free = [](ComponentPool& pool) { delete static_cast<PackedMapWithSparseSearch<T>*>(pool.data); };
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
                        create(e, end);
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
                type_pool.second.remove(type_pool.second, entity);
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
                    type_pool.remove(type_pool, *e);
                this->entities[*e].flags = EntityFlag_TOOMBSTONE;
                available_entities.push_back(*e);
            }
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
            return get_or_create_type_pool_map<T>().emplace(entity, args...);
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
            for (auto e = begin; e < end; e++)
                map.emplace(*e, args...);
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
            const PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.get(entity);
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
                if (!type_pool.has(type_pool, entity))
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

            const PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return const_cast<T*>(map.try_get(entity));
        }
        
        /**
         * @brief Remove a component from an entity and call its destructor
         * 
         * @tparam T The type of component to remove
         * @param entity The ID of the entity to remove the component from
         */
        template<typename T>
        void remove(const Entity entity) {
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
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
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            for (auto e = begin; e != end; e++)
                map.destroy(*e);
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

            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
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
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            for (auto e = begin; e != end; e++) {
                if (this->entities[*e].flags & EntityFlag_TOOMBSTONE)
                    continue;

                map.try_destroy(*e);
            }
        }

        ~MgmEcs() {
        }
    };
}
