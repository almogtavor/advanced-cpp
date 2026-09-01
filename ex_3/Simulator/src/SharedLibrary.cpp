#include "Simulator/SharedLibrary.h"

#include <dlfcn.h>
#include <utility>

namespace simulator {

SharedLibrary::SharedLibrary(const std::string& path) {
    dlerror();

    handle_ = dlopen(path.c_str(), RTLD_NOW);

    if (!handle_) {
        const char* err = dlerror();
        error_ = err ? err : "Unknown dlopen error";
    }
}

SharedLibrary::~SharedLibrary() {
    if (handle_) {
        dlclose(handle_);
    }
}

SharedLibrary::SharedLibrary(SharedLibrary&& other) noexcept
    : handle_(other.handle_),
      error_(std::move(other.error_)) {
    other.handle_ = nullptr;
}

SharedLibrary& SharedLibrary::operator=(SharedLibrary&& other) noexcept {
    if (this != &other) {
        if (handle_) {
            dlclose(handle_);
        }

        handle_ = other.handle_;
        error_ = std::move(other.error_);
        other.handle_ = nullptr;
    }

    return *this;
}

bool SharedLibrary::loaded() const {
    return handle_ != nullptr;
}

const std::string& SharedLibrary::error() const {
    return error_;
}

} // namespace simulator