// Process-test instrumentation: count TCP connection attempts without logging
// per attempt or adding probes to the product. One fixed mmap belongs to each
// child; the harness reads it before and after an outage.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

static uint64_t* attempts;
static int (*real_connect)(int, const struct sockaddr*, socklen_t);

__attribute__((constructor)) static void initialize(void)
{
  real_connect = dlsym(RTLD_NEXT, "connect");
  const char* path = getenv("XGC_TEST_CONNECT_COUNTER");
  if (!path) return;
  const int fd = open(path, O_RDWR);
  if (fd < 0) return;
  void* memory = mmap(NULL, sizeof(uint64_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (memory != MAP_FAILED) attempts = memory;
}

int connect(int fd, const struct sockaddr* address, socklen_t length)
{
  if (attempts && address && (address->sa_family == AF_INET || address->sa_family == AF_INET6)) {
    __atomic_fetch_add(attempts, 1, __ATOMIC_RELAXED);
  }
  return real_connect(fd, address, length);
}
