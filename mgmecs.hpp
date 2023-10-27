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

        EntityReference(class MgmEcs& ecs, const Entity entity) : num_refs{new uint32_t{1}}, ecs{&ecs}, entity{entity} {}

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
        MgmEcs* ecs = nullptr;
        Entity entity = null;

        bool is_valid() const { return ecs != nullptr; }
        template<typename T> T& get();
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

        ComponentReference(class MgmEcs& ecs, const Entity entity, const uint32_t type_id) {
            num_refs = new uint32_t{1};
            this->ecs = &ecs;
            this->entity = entity;
            this->type_id = type_id;
        }

        public:
        uint32_t type_id = null;

        template<typename T> T& get();
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
                Component value;
                bool registered = false;
            };
            struct Page {
                PageMember* data = nullptr;
                uint32_t num_registered = 0;
                Entity id = 0;
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
                    if (pages[i].id < page_id) {
                        left = i;
                        i = (i + right + 1) / 2;
                    }
                    if (pages[i].id == page_id)
                        return PageIndex{ false, i };
                } while (right - left > 1);
                return PageIndex{ true, left };
            }
            Page& get_page(const uint32_t page_id) {
                if (pages.empty())
                    throw std::runtime_error("No component of this type exists");

                if (pages.front().id == page_id) return pages.front();
                if (pages.back().id == page_id) return pages.back();

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point)
                    return pages[page.pos];

                throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
            }
            const Page& get_page(const uint32_t page_id) const {
                if (pages.empty())
                    throw std::runtime_error("No component of this type exists");

                if (pages.front().id == page_id) return pages.front();
                if (pages.back().id == page_id) return pages.back();

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point)
                    return pages[page.pos];

                throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
            }
            Page& get_or_create_page(const uint32_t page_id) {
                if (pages.empty())
                    return pages.emplace_back(Page{std::allocator<PageMember>{}.allocate(S), page_id});

                if (pages.front().id == page_id) return pages.front();
                if (pages.back().id == page_id) return pages.back();

                const PageIndex page = try_get_page_index(page_id);
                if (!page.is_insert_point)
                    return pages[page.pos];

                pages.emplace(pages.begin() + page.pos + 1, Page{std::allocator<PageMember>{}.allocate(S), page_id});
                return pages[page_id];
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

            PackedMapWithSparseSearch() {}

            template<typename... Ts>
            Type& get_or_emplace(const Entity entity, Ts&&... args) {
                const uint32_t page_id = entity / S;
                Page& page = get_or_create_page(page_id);
                PageMember& component = page.data[entity - page_id * S];
                if (component.registered)
                    return data[component.value].second;
                if constexpr (sizeof...(args) > 0)
                    data.emplace_back(entity, args...);
                else
                    data.emplace_back(entity, Type{});
                component.value = data.size() - 1;
                component.registered = true;
                page.num_registered++;
                return data[component.value].second;
            }

            const Type& get(const Entity entity) const {
                const uint32_t page_id = entity / S;
                const Page& page = get_page(page_id);
                const PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                return data[component.value].second;
            }
            Type& get(const Entity entity) {
                const uint32_t page_id = entity / S;
                const Page& page = get_page(page_id);
                const PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                return data[component.value].second;
            }

            const Type* try_get(const Entity entity) const {
                const uint32_t page_id = entity / S;
                const PageIndex page_index = try_get_page_index(page_id);
                if (page_index.is_insert_point)
                    return nullptr;
                const Page& page = pages[page_index.pos];
                const PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    return nullptr;
                return &data[component.value].second;
            }
            Type* try_get(const Entity entity) {
                const uint32_t page_id = entity / S;
                const PageIndex page_index = try_get_page_index(page_id);
                if (page_index.is_insert_point)
                    return nullptr;
                const Page& page = pages[page_index.pos];
                const PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    return nullptr;
                return &data[component.value].second;
            }

            template<typename... Ts>
            Type& emplace(const Entity entity, Ts&&... args) {
                const uint32_t page_id = entity / S;
                Page& page = get_or_create_page(page_id);
                PageMember& component = page.data[entity - page_id * S];
                if (component.registered)
                    throw std::runtime_error("Entity already exists");
                if constexpr (sizeof...(args) > 0)
                    data.emplace_back(entity, args...);
                else
                    data.emplace_back(entity, Type{});
                component.value = data.size() - 1;
                component.registered = true;
                page.num_registered++;
                return data[component.value].second;
            }

            void destroy(Entity entity) {
                uint32_t page_id = entity / S;
                Page& page = get_page(page_id);
                PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    throw std::runtime_error("Entity doesn't exist, or isn't registered for this component type");
                if (component.value != data.size() - 1)
                    std::swap(data[component.value], data.back());
                data.pop_back();

                entity = data[component.value].first;
                page_id = entity / S;
                Page& old_page = get_page(page_id);
                PageMember& old_component = page.data[entity - page_id * S];
                old_component.value = component.value;

                component.value = 0;
                component.registered = false;
                page.num_registered--;
            }
            void try_destroy(Entity entity) {
                uint32_t page_id = entity / S;
                Page& page = get_page(page_id);
                PageMember& component = page.data[entity - page_id * S];
                if (!component.registered)
                    return;
                if (component.value != data.size() - 1)
                    std::swap(data[component.value], data.back());
                data.pop_back();

                entity = data[component.value].first;
                page_id = entity / S;
                Page& old_page = get_page(page_id);
                PageMember& old_component = page.data[entity - page_id * S];
                old_component.value = component.value;

                component.value = 0;
                component.registered = false;
                page.num_registered--;
            }

            ~PackedMapWithSparseSearch() {
                for (auto& page : pages) {
                    for (uint32_t i = 0; i < S; i++)
                        if (page.data[i].registered)
                            page.data[i].value.~Component();
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
            uint32_t type_id = 0;

            std::function<void(const Entity)> remove{};
            std::function<void(const Entity)> try_remove{};
            std::function<bool(const Entity)> has{};

            template<typename T>
            PackedMapWithSparseSearch<T>& map() const {
                return *reinterpret_cast<PackedMapWithSparseSearch<T>*>(data);
            }
        };
        template<typename T>
        PackedMapWithSparseSearch<T>& get_type_pool_map() {
            auto it = type_pools.find(type_id<T>);
            if (it == type_pools.end()) {
                create_type_pool<T>();
                it = type_pools.find(type_id<T>);
            }
            ComponentPool& pool = it->second;
            if (pool.type_id != type_id<T>)
                throw std::runtime_error("INTERNAL: Getting wrong pool type");
            return pool.map<T>();
        }
        template<typename T>
        const PackedMapWithSparseSearch<T>& get_type_pool_map() const {
            auto it = type_pools.find(type_id<T>);
            if (it == type_pools.end())
                throw std::runtime_error("Tried to access unregistered type");
            const ComponentPool& pool = it->second;
            if (pool.type_id != type_id<T>)
                throw std::runtime_error("INTERNAL: Getting wrong pool type");
            return pool.map<T>();
        }
        std::unordered_map<uint32_t, ComponentPool> type_pools{};

        std::vector<EntityInfo> entities{};
        std::vector<Entity> available_entities{};

        template<typename T>
        uint32_t create_type_pool() {
            ComponentPool& pool = type_pools[type_id<T>];

            pool.data = new PackedMapWithSparseSearch<T>;
            pool.type_id = type_id<T>;
            pool.remove = [pool](const Entity entity) { pool.map<T>().destroy(entity); };
            pool.try_remove = [pool](const Entity entity) { pool.map<T>().try_destroy(entity); };
            pool.has = [this](const Entity entity) { return try_get<T>(entity) != nullptr; };
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
        EntityReference entity_reference(const Entity entity) {
            return EntityReference{*this, entity};
        }

        /**
         * @brief Get all entities
         * 
         * @return A vector with all entities
         */
        std::vector<Entity> all() {
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
                type_pool.second.remove(entity);
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
                    type_pool.try_remove(e);
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
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.emplace(entity, args...);
        }

        /**
         * @brief Get a reference to a component
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A reference to the component
         */
        template<typename T>
        const T& get(const Entity entity) const {
            const PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.get(entity);
        }

        /**
         * @brief Get a reference to a component
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A reference to the component
         */
        template<typename T>
        T& get(const Entity entity) {
            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.get(entity);
        }

        /**
         * @brief Get all components connected to an entity
         * 
         * @param entity The ID of the entity to get
         * @return A vector with Component References to all components
         */
        std::vector<ComponentReference> get_all(const Entity entity) {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                throw std::runtime_error("Entity doesn't exist");

            std::vector<ComponentReference> components{};
            for (const auto& [type_id, type_pool] : type_pools) {
                if (!type_pool.has(entity))
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
            return get_type_pool_map<T>().get_or_emplace(entity, args...);
        }

        /**
         * @brief Get a component from an entity
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A pointer to the component, or nullptr if it doesn't exist
         */
        template<typename T>
        const T* try_get(const Entity entity) const {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                return nullptr;

            const PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.try_get(entity);
        }

        /**
         * @brief Get a component from an entity
         * 
         * @tparam T The type of component to get
         * @param entity The ID of the entity to get the component from
         * @return A pointer to the component, or nullptr if it doesn't exist
         */
        template<typename T>
        T* try_get(const Entity entity) {
            if (entities[entity].flags & EntityFlag_TOOMBSTONE)
                return nullptr;

            PackedMapWithSparseSearch<T>& map = get_type_pool_map<T>();
            return map.try_get(entity);
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

        ~MgmEcs() {}
    };

    template<typename T>
    T& EntityReference::get() {
        if (ecs == nullptr)
            throw std::runtime_error("Invalid reference, or wrong type");
        auto& map = ecs->get_type_pool_map<T>();
        return map.get(entity);
    }
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
        ecs->destroy(entity);
        invalidate();
    }

    template<typename T>
    T& ComponentReference::get() {
        if (ecs == nullptr || type_id != MgmEcs::type_id<T>)
            throw std::runtime_error("Invalid reference, or wrong type");
        auto& map = ecs->get_type_pool_map<T>();
        return map.get(entity);
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
        ecs->type_pools[type_id].remove(entity);
        invalidate();
    }
}
