#ifndef EP_COMMON__NODE_ID_HPP
#define EP_COMMON__NODE_ID_HPP

#include <cstdint>
#include <string>

namespace ep_common {

class NodeId
{
protected:
    const uint8_t node_id;

public:
    NodeId(const uint8_t node_id);

    std::string get_formatted_topic(const std::string& base) const;
};

}   // namespaces

#endif  // #ifndef EP_COMMON__NODE_ID_HPP
