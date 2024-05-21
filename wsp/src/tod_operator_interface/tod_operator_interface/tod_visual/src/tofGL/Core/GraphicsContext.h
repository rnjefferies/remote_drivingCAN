// Copyright 2020 TUMFTM based on Cherno's Hazel
#pragma once
#include <memory>

class GraphicsContext {
public:
    virtual ~GraphicsContext() = default;
    virtual void init() = 0;
    virtual void swapBuffers() = 0;
    static std::unique_ptr<GraphicsContext> create(void *window);
};
