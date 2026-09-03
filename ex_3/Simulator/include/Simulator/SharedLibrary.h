#pragma once

#include <string>

namespace simulator {

class SharedLibrary {
public:
    explicit SharedLibrary(const std::string& path);
    ~SharedLibrary();

    // so for SharedLibrary a("plugin.so");   // a.handle_ = 0x7f...
    // SharedLibrary b = a;            // b.handle_ = 0x7f...  same value!
    // end of scope: ~b calls dlclose(0x7f...), then ~a calls dlclose(0x7f...) again
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