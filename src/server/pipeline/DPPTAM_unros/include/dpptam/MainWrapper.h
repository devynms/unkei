#include <boost/thread.hpp>
#include <stdlib.h>
#include <stdint.h>

class MainWrapper {
    public:
    MainWrapper():done(false){}
    bool done;
    boost::mutex done_mutex;
    boost::condition_variable done_cond;
};
