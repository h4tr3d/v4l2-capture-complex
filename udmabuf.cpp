#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cstdlib>
#include <cstring>

#include <fstream>
#include <iostream>

#include <fmt/format.h>

#include "udmabuf.h"

namespace {

const char* dev_prefix = "/dev/";
const char* sys_prefix = "/sys/class/u-dma-buf/";

}

udmabuf::udmabuf()
{
}

udmabuf::udmabuf(std::string_view name)
    : m_name(name),
      m_dev_name(dev_prefix),
      m_class_path(sys_prefix)
{
    m_dev_name.append(name);
    m_class_path.append(name);

    get_prop("phys_addr") >> std::hex >> m_phys_addr;
    get_prop("size") >> std::dec >> m_size;

    std::cout << fmt::format("addr: 0x{0:0{1}X}, size: {2}", m_phys_addr, sizeof(m_phys_addr)*2, m_size) << '\n';

    map();
}

udmabuf::~udmabuf()
{
    if (m_ptr && m_ptr != MAP_FAILED) {
        munmap(m_ptr, m_size);
    }
    ::close(m_fd);
}

void udmabuf::sync_for_cpu(unsigned long offset, unsigned long length) const
{
    char attr[1024];
    const unsigned long  sync_offset = offset;
    const unsigned long  sync_size = length;
    unsigned int   sync_direction = 1;
    unsigned long  sync_for_cpu   = 1;
    auto path = m_class_path + "/sync_for_cpu";
    if (int fd; (fd  = open(path.c_str(), O_WRONLY)) != -1) {
        snprintf(attr, sizeof(attr), "0x%08X%08X", (sync_offset & 0xFFFFFFFF), (sync_size & 0xFFFFFFF0) | (sync_direction << 2) | sync_for_cpu);
        write(fd, attr, strlen(attr));
        close(fd);
    }
}

const void *udmabuf::get() const
{
    return m_ptr;
}

void *udmabuf::get()
{
    return m_ptr;
}

size_t udmabuf::size() const
{
    return m_size;
}

void udmabuf::map()
{
    // TBD: optional O_SYNC
    if (int fd; (fd = open(m_dev_name.c_str(), O_RDWR)) != -1) {
        m_ptr = mmap(nullptr, m_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        m_fd = fd;
        if (m_ptr == MAP_FAILED) {
            std::error_code ec{errno, std::system_category()};
            throw std::system_error(ec, fmt::format("Can't map CMA memory"));
        }
        fmt::print("cma vaddr: {0}\n", m_ptr);
    }
}
