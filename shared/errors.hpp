#pragma once

#include <cassert>
#include <string>
#include <system_error>

// https://github.com/cryptocode/raiblocks/wiki/Error-handling-in-the-node-using-std::error_code-and-std::expected
// Convenience macro to implement the standard boilerplate for using std::error_code with enums
// Use this at the end of any header defining one or more error code enums.
#define REGISTER_ERROR_CODES(namespace_name, enum_type)                                                      \
    namespace namespace_name                                                                                 \
    {                                                                                                        \
        static_assert(static_cast<int>(enum_type::generic) > 0, "The first error enum must be generic = 1"); \
        class enum_type##_messages : public std::error_category                                              \
        {                                                                                                    \
        public:                                                                                              \
            const char* name() const noexcept override                                                       \
            {                                                                                                \
                return #enum_type;                                                                           \
            }                                                                                                \
                                                                                                             \
            std::string message(int ev) const override;                                                      \
        };                                                                                                   \
                                                                                                             \
        inline const std::error_category& enum_type##_category()                                             \
        {                                                                                                    \
            static enum_type##_messages instance;                                                            \
            return instance;                                                                                 \
        }                                                                                                    \
                                                                                                             \
        inline std::error_code make_error_code(::namespace_name::enum_type err)                              \
        {                                                                                                    \
            return {static_cast<int>(err), enum_type##_category()};                                          \
        }                                                                                                    \
    }                                                                                                        \
    namespace std                                                                                            \
    {                                                                                                        \
        template <>                                                                                          \
        struct is_error_code_enum<::namespace_name::enum_type> : public std::true_type                       \
        {                                                                                                    \
        };                                                                                                   \
    }
