#pragma once
#include <atomic>
#include <cstdint>
#include <memory.h>
#include <functional>
#include <unordered_map>
#include <stdexcept>
#include <vector>


namespace mgm {
    using Entity = uint32_t;
    using Component = uint32_t;
    constexpr Entity null = (Entity)-1;
    constexpr Component null_component = (Component)-1;
    constexpr uint32_t null_type_id = (uint32_t)-1;

    struct EntityReference {
        private:
        friend class MgmEcs;

        EntityReference(const class MgmEcs& ecs, const Entity entity) : num_refs{new uint32_t{1}}, ecs{&ecs}, entity{entity} {}

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
        template<typename T> const T& get() const;
        void destroy_original();

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

        ComponentReference(const class MgmEcs& ecs, const Entity entity, const uint32_t type_id) {
            num_refs = new uint32_t{1};
            this->ecs = &ecs;
            this->entity = entity;
            this->type_id = type_id;
        }

        public:
        uint32_t type_id = null;

        template<typename T> const T& get() const;
        void destroy_original();

        ~ComponentReference() {
            invalidate();
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

        private:
        template<typename Type, size_t S = 256>
        class PackedMapWithSparseSearch {
            struct PageMember {
                Component value : (sizeof(Component) - 1);
                bool registered : 1 = false;
            };
            struct Page {
                PageMember* data = nullptr;
                uint32_t id = 0;
            };
            std::vector<Page> pages{};
            std::vector<std::pair<Entity, Type>> data{};

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
                return PageIndex{ true, left };
            }
            Page& get_page(const uint32_t page_id) const {
                if (pages.front().id == page_id) return const_cast<Page&>(pages.front());
                if (pages.back().id == page_id) return const_cast<Page&>(pages.back());

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point)
                    return const_cast<Page&>(pages[page.pos]);

                throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
            }
            Page& get_or_create_page(const uint32_t page_id) {
                if (pages.front().id == page_id) return pages.front();
                else if (pages.back().id == page_id) return pages.back();

                PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point)
                    return pages[page.pos];

                page.pos++;
                pages.emplace(pages.begin() + page.pos, Page{new PageMember[S]{}, page_id});
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
                data.reserve(S);
                pages.emplace_back(Page{new PageMember[S]{}, 0});
            }

            template<typename... Ts>
            Type& emplace(const Entity entity, Ts&&... args) {
                PageMember& component = get_or_create_page(entity / S).data[entity % S];
                if (component.registered)
                    throw std::runtime_error("Entity already exists");
                component.value = data.size();
                component.registered = true;
                if constexpr (sizeof...(args) > 0)
                    data.emplace_back(entity, args...);
                else
                    data.emplace_back(entity, Type{});
                return data[component.value].second;
            }

            template<typename... Ts>
            Type& get_or_emplace(const Entity entity, Ts&&... args) {
                PageMember& component = get_or_create_page(entity / S).data[entity % S];
                if (component.registered)
                    return data[component.value].second;
                component.value = data.size();
                component.registered = true;
                if constexpr (sizeof...(args) > 0)
                    data.emplace_back(entity, args...);
                else
                    data.emplace_back(entity, Type{});
                return data[component.value].second;
            }

            Type& get(const Entity entity) const {
                const PageMember& component = get_page(entity / S).data[entity % S];
                if (!component.registered)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                return *const_cast<Type*>(&data[component.value].second);
            }

            Type* try_get(const Entity entity) const {
                const PageIndex page_index = try_get_page_index(entity / S);
                if (page_index.is_insert_point)
                    return nullptr;
                const Page& page = pages[page_index.pos];
                const PageMember& component = page.data[entity % S];
                if (!component.registered)
                    return nullptr;
                return const_cast<Type*>(&data[component.value].second);
            }

            void destroy(const Entity entity) {
                PageMember& component = get_page(entity / S).data[entity % S];
                if (!component.registered)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                if (component.value != data.size() - 1)
                    std::swap(data[component.value], data.back());
                data.pop_back();

                get_page(data[component.value].first / S).data[data[component.value].first % S].value = component.value;

                component.value = 0;
                component.registered = false;
            }
            void try_destroy(Entity entity) {
                PageMember& component = get_page(entity / S).data[entity % S];
                if (!component.registered)
                    return;
                if (component.value != data.size() - 1)
                    std::swap(data[component.value], data.back());
                data.pop_back();

                get_page(data[component.value].first / S).data[data[component.value].first % S].value = component.value;

                component.value = 0;
                component.registered = false;
            }

            ~PackedMapWithSparseSearch() {
                for (auto& page : pages) {
                    delete[] page.data;
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
            if (it == type_pools.end()) {
                create_type_pool<T>();
                it = type_pools.find(type_id<T>);
            }
            ComponentPool& pool = it->second;
            return pool.map<T>();
        }
        template<typename T>
        PackedMapWithSparseSearch<T>& get_type_pool_map() const {
            auto it = type_pools.find(type_id<T>);
            if (it == type_pools.end())
                throw std::runtime_error("Tried to access unregistered type");
            const ComponentPool& pool = it->second;
            return const_cast<PackedMapWithSparseSearch<T>&>(pool.map<T>());
        }
        std::unordered_map<uint32_t, ComponentPool> type_pools{};

        std::vector<EntityInfo> entities{};
        std::vector<Entity> available_entities{};

        template<typename T>
        uint32_t create_type_pool() {
            ComponentPool& pool = type_pools[type_id<T>];

            pool.data = new PackedMapWithSparseSearch<T>;
            pool.remove = [](ComponentPool& pool, const Entity entity) { pool.map<T>().destroy(entity); };
            pool.try_remove = [](ComponentPool& pool, const Entity entity) { pool.map<T>().try_destroy(entity); };
            pool.has = [](const ComponentPool& pool, const Entity entity) { return pool.map<T>().try_get(entity) != nullptr; };
            pool.free = [](ComponentPool& pool) { delete static_cast<PackedMapWithSparseSearch<T>*>(pool.data); };
            return type_id<T>;
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
        std::vector<Entity> create(const uint32_t num) {
            std::vector<Entity> new_entity_ids{};
            new_entity_ids.reserve(num);

            if (available_entities.empty()) {
                Entity num_entities = (Entity)entities.size();
                entities.resize(entities.size() + num);
                for (auto e = entities.end() - num; e != entities.end(); e++) {
                    auto& e_id = *e;
                    e_id = EntityInfo{};
                    new_entity_ids.emplace_back(num_entities++);
                }
            }
            else if (num <= available_entities.size()) {
                for (auto e = available_entities.end() - num; e != available_entities.end(); e++) {
                    const auto& e_id = *e;
                    new_entity_ids.emplace_back(e_id);
                    entities[e_id] = EntityInfo{};
                }
                available_entities.erase(available_entities.end() - num, available_entities.end());
            }
            else {
                for (const auto& e_id : available_entities) {
                    new_entity_ids.emplace_back(e_id);
                    entities[e_id] = EntityInfo{};
                }
                const size_t remaining_entities = num - available_entities.size();
                available_entities.clear();
                const auto next_new_entity_ids = create(remaining_entities);
                new_entity_ids.insert(new_entity_ids.end(), next_new_entity_ids.begin(), next_new_entity_ids.end());
            }
            return new_entity_ids;
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
         * @brief Get all entities
         * 
         * @return A vector with all entities
         */
        std::vector<Entity> all() const {
            std::vector<Entity> res{};
            res.reserve(entities.size());
            for (Entity i = 0; i < entities.size(); i++)
                if (!(entities[i].flags & EntityFlag_TOOMBSTONE))
                    res.emplace_back(i);
            return res;
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
         * @param entities A vector with all entity IDs to destroy
         */
        void destroy(const std::vector<Entity> entities) {
            for (const auto& e : entities) {
                for (auto& [tid, type_pool] : type_pools)
                    type_pool.try_remove(type_pool, e);
                this->entities[e].flags = EntityFlag_TOOMBSTONE;
                available_entities.push_back(e);
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
         * @param entities A vector with all entities to remove the component from
         */
        template<typename T>
        void remove(const std::vector<Entity>& entities) {
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            for (const auto& e : entities)
                map.destroy(e);
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
         * @param entities A vector containing the IDs of the entities to remove the component from
         */
        template<typename T>
        void try_remove(const std::vector<Entity>& entities) {
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            for (const auto& e : entities) {
                if (this->entities[e].flags & EntityFlag_TOOMBSTONE)
                    continue;

                map.try_destroy(e);
            }
        }

        ~MgmEcs() {
            destroy(all());
        }
    };

    template<typename T>
    const T& EntityReference::get() const {
        if (ecs == nullptr)
            throw std::runtime_error("Invalid reference, or wrong type");
        const auto& map = ecs->get_type_pool_map<T>();
        return map.get(entity);
    }

    inline void EntityReference::destroy_original() {
        if (ecs == nullptr)
            throw std::runtime_error("Invalid reference");
        if (*num_refs > 1)
            throw std::runtime_error("Other references are using this entity, so it cannot be destroyed");
        const_cast<MgmEcs*>(ecs)->destroy(entity);
        invalidate();
    }

    template<typename T>
    const T& ComponentReference::get() const {
        if (ecs == nullptr || type_id != MgmEcs::type_id<T>)
            throw std::runtime_error("Invalid reference, or wrong type");
        const auto& map = ecs->get_type_pool_map<T>();
        return map.get(entity);
    }

    inline void ComponentReference::destroy_original() {
        if (ecs == nullptr)
            throw std::runtime_error("Invalid reference");
        if (*num_refs > 1)
            throw std::runtime_error("Other references are using this component, so it cannot be destroyed");
        MgmEcs::ComponentPool& type_pool = const_cast<MgmEcs*>(ecs)->type_pools[type_id];
        type_pool.remove(type_pool, entity);
        invalidate();
    }
}
