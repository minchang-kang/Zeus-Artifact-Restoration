// Stub implementation of noether::Factory for ROS-free builds.
// The plugin-based Factory constructor is never called; only the direct-object
// constructor of ToolPathPlannerPipeline is used. These stubs satisfy the linker.
#include <noether_tpp/plugin_interface.h>
#include <stdexcept>

namespace noether
{

Factory::Factory() = default;

Factory::Factory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader)
    : loader_(std::move(loader))
{
}

MeshModifier::Ptr Factory::createMeshModifier(const YAML::Node&) const
{
    throw std::logic_error("Factory::createMeshModifier: plugin system not compiled");
}

ToolPathPlanner::Ptr Factory::createToolPathPlanner(const YAML::Node&) const
{
    throw std::logic_error("Factory::createToolPathPlanner: plugin system not compiled");
}

DirectionGenerator::Ptr Factory::createDirectionGenerator(const YAML::Node&) const
{
    throw std::logic_error("Factory::createDirectionGenerator: plugin system not compiled");
}

OriginGenerator::Ptr Factory::createOriginGenerator(const YAML::Node&) const
{
    throw std::logic_error("Factory::createOriginGenerator: plugin system not compiled");
}

ToolPathModifier::Ptr Factory::createToolPathModifier(const YAML::Node&) const
{
    throw std::logic_error("Factory::createToolPathModifier: plugin system not compiled");
}

}  // namespace noether
