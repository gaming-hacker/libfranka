// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <Poco/TemporaryFile.h>
#include <string>
#include "network.h"

namespace franka {

class LibraryDownloader {
 public:
  explicit LibraryDownloader(Network& network);
  ~LibraryDownloader();

  [[nodiscard]] auto path() const noexcept -> const std::string& ;

 private:
  Poco::File model_library_file_;
};

};  // namespace franka
