#pragma once

#include <future>
#include <map>
#include <string>
#include <vector>

enum ParamKeyType {
  PERSISTENT = 0x02,
  CLEAR_ON_MANAGER_START = 0x04,
  CLEAR_ON_ONROAD_TRANSITION = 0x08,
  CLEAR_ON_OFFROAD_TRANSITION = 0x10,
  DONT_LOG = 0x20,
  ALL = 0xFFFFFFFF
};

class Params {
public:
  explicit Params(const std::string &path = {});
  // Not copyable.
  Params(const Params&) = delete;
  Params& operator=(const Params&) = delete;

  std::vector<std::string> allKeys() const;
  bool checkKey(const std::string &key);
  ParamKeyType getKeyType(const std::string &key);
  inline std::string getParamPath(const std::string &key = {}) {
    return params_path + prefix + (key.empty() ? "" : "/" + key);
  }

  // Delete a value
  int remove(const std::string &key);
  void clearAll(ParamKeyType type);

  // helpers for reading values
  std::string get(const std::string &key, bool block = false);
  inline bool getBool(const std::string &key, bool block = false) {
    return get(key, block) == "1";
  }
  inline int getInt(const std::string &key, bool block = false) {
    std::string value = get(key, block);
    return value.empty() ? 0 : std::stoi(value);
  }
  std::map<std::string, std::string> readAll();

  // helpers for writing values
  int put(const char *key, const char *val, size_t value_size);
  inline int put(const std::string &key, const std::string &val) {
    return put(key.c_str(), val.data(), val.size());
  }
  inline int putBool(const std::string &key, bool val) {
    return put(key.c_str(), val ? "1" : "0", 1);
  }
  inline int putInt(const std::string &key, int val) {
    return put(key.c_str(), std::to_string(val).c_str(), std::to_string(val).size());
  }
  inline void putBoolNonBlocking(const std::string &key, bool val) {
    std::async(std::launch::async, [this, &key, val] { put(key, val ? "1" : "0"); }).get();
  }
  inline void putIntNonBlocking(const std::string &key, int val) {
    std::async(std::launch::async, [this, &key, val] { put(key, std::to_string(val)); }).get();
  }

private:
  std::string params_path;
  std::string prefix;
};
