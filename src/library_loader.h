// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <Poco/SharedLibrary.h>
#include <string>

namespace franka {

/*
 * Wraps library loading and unloading with RAII.
 */
class LibraryLoader {
 public:
  explicit LibraryLoader(const std::string& filepath);
  ~LibraryLoader();

  auto getSymbol(const std::string& symbol_name) -> void* ;

 private:
  Poco::SharedLibrary library_;
};

}  // namespace franka
