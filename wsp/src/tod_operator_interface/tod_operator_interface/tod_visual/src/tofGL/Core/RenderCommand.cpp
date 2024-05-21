// Copyright 2021 TUMFTM
#include "Core/RenderCommand.h"
std::unique_ptr<RendererAPI> RenderCommand::s_RendererAPI = RendererAPI::Create();
