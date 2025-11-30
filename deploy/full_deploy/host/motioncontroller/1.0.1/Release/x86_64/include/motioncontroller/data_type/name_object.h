#pragma once
#include <string>
namespace mc {
namespace data_type {
class NameObject {
 public:
  NameObject(std::string id);
  std::string GetID();
  std::string GetName();
  void SetName(std::string name);

 protected:
  std::string id_;
  std::string name_{"default name"};
};
}  // namespace data_type
}  // namespace mc