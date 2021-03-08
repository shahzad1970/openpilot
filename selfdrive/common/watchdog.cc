#include <string>
#include <cstdint>
#include <unistd.h>

#include "common/watchdog.h"
#include "common/util.h"
#include "common/timing.h"

const std::string watchdog_fn = "/dev/shm/wd_";  // + <pid>

bool watchdock_kick(){
    std::string fn = watchdog_fn + std::to_string(getpid());
    std::string cur_t = std::to_string(nanos_since_boot());

    int r = write_file(fn.c_str(), cur_t.data(), cur_t.length(), O_WRONLY | O_CREAT);
    return r == 0;
}