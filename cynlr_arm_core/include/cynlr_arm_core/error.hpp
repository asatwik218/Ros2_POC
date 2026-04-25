#pragma once
#include <string>
#include <tl/expected.hpp>

namespace cynlr::arm {

enum class ErrorCode {
    OK = 0,
    NOT_CONNECTED,
    NOT_ENABLED,
    NOT_OPERATIONAL,
    FAULT,
    INVALID_ARGUMENT,
    TIMEOUT,
    MODE_SWITCH_FAILED,
    MOTION_FAILED,
    SDK_EXCEPTION,
    UNSUPPORTED,
    UNKNOWN,
};

struct Error {
    ErrorCode code{ErrorCode::UNKNOWN};
    std::string message;
};

template <typename T>
using Expected = tl::expected<T, Error>;

inline auto make_error(ErrorCode code, std::string msg) {
    return tl::unexpected(Error{code, std::move(msg)});
}

} // namespace cynlr::arm
