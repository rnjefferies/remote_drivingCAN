// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include <utility>
#include <entt/entt.hpp>
#include "Scene.h"

class Entity {
public:
    Entity() = default;
    Entity(const Entity &entity) = default;
    Entity(const entt::entity &handle, Scene *scene) : m_EntityHandle(handle), m_Scene(scene) {}

    entt::entity& GetHandle() { return m_EntityHandle; }

    template <typename T, typename... Args>
    T &AddComponent(Args &&... args) {
        return m_Scene->_registry.emplace<T>(m_EntityHandle, std::forward<Args>(args)...);
    }

    template <typename T>
    T &GetComponent() { return m_Scene->_registry.get<T>(m_EntityHandle); }

    template <typename T>
    bool HasComponent() { return m_Scene->_registry.has<T>(m_EntityHandle); }

    template <typename T>
    void RemoveComponent() { m_Scene->_registry.remove<T>(m_EntityHandle); }

    operator bool() const { return m_EntityHandle != entt::null; }

private:
    entt::entity m_EntityHandle{entt::null};
    Scene *m_Scene{nullptr};
};
