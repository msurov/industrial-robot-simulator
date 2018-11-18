#include "signals.h"
#include <signal.h>


namespace signals
{
    static sighandler_t _sigint_handler;
    static sighandler_t _sigterm_handler;
    static sighandler_t _sighup_handler;


    void handle(int sig)
    {
        switch (sig)
        {
        case SIGTERM:
            if (_sigterm_handler)
                _sigterm_handler();
            break;
        case SIGINT:
            if (_sigint_handler)
                _sigint_handler();
            break;
        case SIGHUP:
            if (_sighup_handler)
                _sighup_handler();
            break;
        }
    }

    void set_sigint_handler(sighandler_t const& handler)
    {
        signal(SIGINT, &handle);
        _sigint_handler = handler;
    }

    void set_sigterm_handler(sighandler_t const& handler)
    {
        signal(SIGTERM, &handle);
        _sigterm_handler = handler;
    }

    void set_sighup_handler(sighandler_t const& handler)
    {
        signal(SIGHUP, &handle);
        _sighup_handler = handler;
    }
}
