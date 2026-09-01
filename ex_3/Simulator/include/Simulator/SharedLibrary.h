#pragma once

#include <string>

namespace simulator {

class SharedLibrary {
public:
    explicit SharedLibrary(const std::string& path);
    ~SharedLibrary();

    SharedLibrary(const SharedLibrary&) = delete;
    SharedLibrary& operator=(const SharedLibrary&) = delete;

    SharedLibrary(SharedLibrary&& other) noexcept;
    SharedLibrary& operator=(SharedLibrary&& other) noexcept;

    bool loaded() const;
    const std::string& error() const;

private:
    void* handle_ = nullptr;
    std::string error_;
};

} // namespace simulator