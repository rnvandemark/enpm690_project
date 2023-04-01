#include <ep_common/node_id.hpp>

namespace ep_common {

NodeId::NodeId(const uint8_t node_id) :
    node_id(node_id)
{
}

std::string NodeId::get_formatted_topic(const std::string& base) const
{
    return get_formatted_topic(base, node_id);
}

std::string NodeId::get_formatted_topic(
    const std::string& base,
    const uint8_t node_id)
{
    return base + "_" + std::to_string(static_cast<int>(node_id));
}

}   // namespaces
