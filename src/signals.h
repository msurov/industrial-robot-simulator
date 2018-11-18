#pragma once

#include <functional>


namespace signals
{
    using sighandler_t = std::function<void(void)>;
    void set_sigint_handler(sighandler_t const& handler);
    void set_sigterm_handler(sighandler_t const& handler);
    void set_sighup_handler(sighandler_t const& handler);
};
