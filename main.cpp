//
// Refer to:
// * https://www.linuxtv.org/downloads/v4l-dvb-apis-new/uapi/v4l/capture.c.html
//

#include <string>
#include <iostream>
#include <vector>
#include <chrono>
#include <system_error>
#include <functional>

#include <cstdio>
#include <cstdlib>
//#include <cstring>
#include <cassert>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <fmt/format.h>

#include "udmabuf.h"

using namespace std::chrono_literals;

namespace v4l2 {

struct format_descriptor
{
    static constexpr uint32_t max_planes = VIDEO_MAX_PLANES;

    struct plane_descriptor
    {
        uint32_t sizeimage = 0;
        uint32_t bytesperline = 0;
    };

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixelformat = 0;
    uint32_t num_planes = 0;
    plane_descriptor plane_descr[max_planes];

    uint32_t get_sizeimage() const
    {
        uint32_t total = 0;
        for (size_t i = 0; i < num_planes; ++i) {
            total += plane_descr[i].sizeimage;
        }
        return total;
    }
};

struct memory
{
    void* start = nullptr;
    size_t bytesused = 0;
    size_t length = 0;
};

enum class frame_status
{
    Ok,
    Timeout,
    Interrupt,
};

using on_read_proc_t = std::function<void(const memory* mem, size_t num_mem)>;

template<typename Deleter>
struct buffer
{
    size_t num_mem = 0;
//    memory mem[format_descriptor::max_planes];
    void* mem[format_descriptor::max_planes];
    v4l2_plane planes[format_descriptor::max_planes];

    buffer() = default;
    buffer(const buffer&) = delete;
    buffer(buffer&&) = default;
    buffer& operator=(const buffer&) = delete;
    buffer& operator=(buffer&&) = default;

    ~buffer()
    {
        Deleter()(mem, planes, num_mem);
    }
};

namespace priv {
static bool xioctl(int fh, int request, void *arg)
{
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    if (r == -1) {
        if (errno == EAGAIN) {
            return false;
        }
        auto const ec = std::error_code(errno, std::system_category());
        throw std::system_error(ec,
                                fmt::format("V4L2 ioctl error ({0})", ec.value()));
    }
    return true;
}

struct file_descriptor_wrapper
{
    int fd = -1;

    file_descriptor_wrapper() = default;
    file_descriptor_wrapper(int fd) : fd(fd) {}
    file_descriptor_wrapper(std::nullptr_t) : fd(-1) {}

    operator int() {return fd;}

    bool operator ==(const file_descriptor_wrapper &other) const {return fd == other.fd;}
    bool operator !=(const file_descriptor_wrapper &other) const {return fd != other.fd;}
    bool operator ==(std::nullptr_t) const {return fd == -1;}
    bool operator !=(std::nullptr_t) const {return fd != -1;}
};

struct file_descriptor_deleter
{
    typedef file_descriptor_wrapper pointer;
    void operator()(pointer descr) const
    {
        ::close(descr.fd);
    }
};

using file_descriptor = std::unique_ptr<file_descriptor_wrapper, file_descriptor_deleter>;

}

template<typename T>
class capture
{
public:

    capture() = default;

    explicit capture(const std::string& device)
        : m_device(device)
    {
        open();
    }

    void init()
    {
        init_common();
        self().get_format();
        self().init_io();
    }

    void init(uint32_t width, uint32_t height, uint32_t pixelformat)
    {
        init_common();
        self().set_format(width, height, pixelformat);
        self().init_io();
    }

    void start()
    {
        self().do_start();
    }

    template<typename Duration>
    frame_status wait_frame(Duration wait) const
    {
        fd_set fds;
        struct timeval tv {};
        int r;

        auto duration_micros = std::chrono::duration_cast<std::chrono::microseconds>(wait);

        FD_ZERO(&fds);
        FD_SET(fd(), &fds);

        /* Timeout. */
        tv.tv_sec = duration_micros.count() / 1'000'000;
        tv.tv_usec = duration_micros.count() % 1'000'000;

        r = select(fd() + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                return frame_status::Interrupt;
            std::error_code ec{errno, std::system_category()};
            throw std::system_error(ec, fmt::format("{0} frame wait error: ({1})",
                                                    m_device, ec.value()));
        }

        if (0 == r) {
            return frame_status::Timeout;
        }

        return frame_status::Ok;
    }

    bool read(const on_read_proc_t& on_read)
    {
        return self().do_read(on_read);
    }

    const std::string& device() const
    {
        return m_device;
    }

    bool is_mplane() const
    {
        return m_mplane;
    }

    int fd() const
    {
        return m_fd.get();
    }

    uint32_t width() const
    {
        return m_format.width;
    }

    uint32_t height() const
    {
        return m_format.height;
    }

    uint32_t pixelformat() const
    {
        return m_format.pixelformat;
    }

private:
    T& self()
    {
        return static_cast<T&>(*this);
    }

    const T& self() const
    {
        return static_cast<const T&>(*this);
    }

    void open()
    {
        m_fd.reset(::open(m_device.c_str(), O_RDWR | O_NONBLOCK, 0));
        if (!m_fd) {
            auto ec = std::error_code(errno, std::system_category());
            throw std::system_error(ec, fmt::format("can't open device '{0}': ({1})",
                                                    m_device,
                                                    ec.value()));
        }
    }

    void init_common()
    {
        v4l2_capability cap;

        priv::xioctl(m_fd.get(), VIDIOC_QUERYCAP, &cap);
        if (!(cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
            throw std::system_error({EINVAL, std::system_category()},
                                    fmt::format("{0} is not video capture device", m_device));
        }

        m_mplane = !!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);

        // May be differ for various type of capturing
        self().check_caps(cap);
    }

    template<typename S>
    void fill_fields(S& stream_format, uint32_t width, uint32_t height, uint32_t fmt)
    {
        static_assert (std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format_mplane> ||
                std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format>, "wrong format type");

        stream_format.width = width;
        stream_format.height = height;
        stream_format.pixelformat = fmt;
    }

    void set_format(uint32_t width, uint32_t height, uint32_t pixelformat)
    {
        v4l2_format fmt {};
        fmt.type =  is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (is_mplane()) {
            fill_fields(fmt.fmt.pix_mp, width, height, pixelformat);
        } else {
            fill_fields(fmt.fmt.pix, width, height, pixelformat);
        }

        priv::xioctl(fd(), VIDIOC_S_FMT, &fmt);

        if (is_mplane()) {
            init_fields(fmt.fmt.pix_mp);
        } else {
            init_fields(fmt.fmt.pix);
        }
    }

    template<typename S>
    void init_fields(const S& stream_format)
    {
        m_format.width = stream_format.width;
        m_format.height = stream_format.height;
        m_format.pixelformat = stream_format.pixelformat;

        static_assert (std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format_mplane> ||
                std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format>, "wrong format type");

        if constexpr (std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format_mplane>) {
            // Mutlti planar
            m_format.num_planes = stream_format.num_planes;
            for (size_t i = 0; i < m_format.num_planes; ++i) {
                m_format.plane_descr[i].sizeimage = stream_format.plane_fmt[i].sizeimage;
                m_format.plane_descr[i].bytesperline = stream_format.plane_fmt[i].bytesperline;
            }
        } else if constexpr (std::is_same_v<std::remove_cv_t<S>, v4l2_pix_format>) {
            // Single planar
            m_format.num_planes = 1;
            m_format.plane_descr[0].sizeimage = stream_format.sizeimage;
            m_format.plane_descr[0].bytesperline = stream_format.bytesperline;
        }
    }

    void get_format()
    {
        v4l2_format fmt {};
        fmt.type =  is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        priv::xioctl(fd(), VIDIOC_G_FMT, &fmt);

        if (is_mplane()) {
            init_fields(fmt.fmt.pix_mp);
        } else {
            init_fields(fmt.fmt.pix);
        }
    }

protected:
    const std::string m_device;
    priv::file_descriptor m_fd;
    bool m_mplane = false;
    format_descriptor m_format;
};





class capture_mmap : public capture<capture_mmap>
{
public:
    using base = capture<capture_mmap>;
    friend class capture<capture_mmap>;

    using base::base;

    explicit capture_mmap(const std::string& device, size_t buffers = 4)
        : base(device)
    {
        m_buffers.reserve(buffers);
    }

private:
    void check_caps(const v4l2_capability &cap)
    {
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw std::system_error({EINVAL, std::system_category()},
                                    fmt::format("{0} does not support streaming i/o", device()));
        }
    }

    void init_io()
    {
        const auto type = is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        const uint32_t memory = V4L2_MEMORY_MMAP;

        v4l2_requestbuffers req {};

        req.count = m_buffers.capacity();
        req.type = type;
        req.memory = V4L2_MEMORY_MMAP;

        priv::xioctl(fd(), VIDIOC_REQBUFS, &req);

        if (req.count < 2) {
            // WRONG CASE!
            // TODO
        }

        for (size_t n = 0; n < req.count; ++n) {
            v4l2_buffer buf {};

            auto& buffer = m_buffers.emplace_back();

            buf.type = type;
            buf.memory = memory;
            buf.index = n;

            if (is_mplane()) {
                buf.length = m_format.num_planes;
                buf.m.planes = buffer.planes;
            }

            priv::xioctl(fd(), VIDIOC_QUERYBUF, &buf);

            const size_t planes = is_mplane() ? buf.length : 1;

            for (size_t plane = 0; plane < planes; ++plane) {
                void *ptr = MAP_FAILED;
                size_t length = 0;
                if (is_mplane()) {
                    length = buf.m.planes[plane].length;
                    ptr = ::mmap(nullptr, length, PROT_READ|PROT_WRITE, MAP_SHARED, fd(), buf.m.planes[plane].m.mem_offset);
                } else {
                    length = buf.length;
                    ptr = ::mmap(nullptr, length, PROT_READ|PROT_WRITE, MAP_SHARED, fd(), buf.m.offset);
                }

                if (MAP_FAILED == ptr) {
                    throw std::system_error({ENOBUFS, std::system_category()}, "Can't MMAP buffer");
                }

                buffer.num_mem++;
                buffer.mem[plane] = ptr;
            }
        }
    }

    void do_start()
    {
        auto type = is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        // Feed buffer intos V4L2 subsystem
        for (size_t i = 0; i < m_buffers.size(); ++i) {
            v4l2_buffer buf {};

            buf.type = type;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (is_mplane()) {
                buf.length = m_format.num_planes;
                buf.m.planes = m_buffers[i].planes;
            }

            priv::xioctl(fd(), VIDIOC_QBUF, &buf);
        }

        // Start
        priv::xioctl(fd(), VIDIOC_STREAMON, &type);
    }

    bool do_read(const on_read_proc_t& on_read)
    {
        const auto type = is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        v4l2_buffer buf;
        v4l2_plane planes[format_descriptor::max_planes] = {};

        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;

        if (is_mplane()) {
            buf.length = m_format.num_planes;
            buf.m.planes = planes;
        }

        if (!priv::xioctl(fd(), VIDIOC_DQBUF, &buf))
            return false;

        if (on_read) {
            memory mem[format_descriptor::max_planes];
            memcpy(mem, m_buffers[buf.index].mem, sizeof(mem));

            if (is_mplane()) {
                for (size_t i = 0; i < buf.length; ++i) {
                    mem[i].bytesused = buf.m.planes[i].bytesused;
                }
                // restore pointers
                buf.m.planes = m_buffers[buf.index].planes;
            } else {
                mem[0].bytesused = buf.bytesused;
            }

            on_read(mem, m_buffers[buf.index].num_mem);
        }

        // return buffer back
        priv::xioctl(fd(), VIDIOC_QBUF, &buf);

        return true;
    }

private:
    struct plane_deleter
    {
        void operator()(void** mem, const v4l2_plane* planes, size_t num_mem)
        {
            for (size_t i = 0; i < num_mem; ++i) {
                ::munmap(mem[i], planes[i].length);
            }
        }
    };

    std::vector<buffer<plane_deleter>> m_buffers;
};





struct simple_frame_allocator
{
    explicit simple_frame_allocator(size_t buffers)
    {

    }

    void set_buffers_count(size_t count)
    {

    }

    void set_num_planes(size_t num_planes)
    {
        plane_size.resize(num_planes);
    }

    void set_plane_size(size_t plane, size_t size)
    {
        plane_size[plane] = size;
    }

    void set_format(uint32_t width, uint32_t height, uint32_t pix_fmt)
    {

    }

    void aquire(size_t index, memory *mem, size_t num_mem)
    {
        for (size_t i = 0; i < num_mem; ++i) {
#if 0
            mem[i].start = malloc(plane_size[i]);
#else
            posix_memalign(&mem[i].start, 4096, plane_size[i]);
#endif
            mem[i].bytesused = 0;
            mem[i].length = plane_size[i];
        }
    }

    void release(size_t index, memory *mem, size_t num_mem)
    {
        for (size_t i = 0; i < num_mem; ++i) {
            free(mem[i].start);
        }
    }

    std::vector<size_t> plane_size;
};




template<typename FrameAllocator = simple_frame_allocator>
class capture_userptr : public capture<capture_userptr<FrameAllocator>>
{
public:
    using base = capture<capture_userptr<FrameAllocator>>;
    friend class capture<capture_userptr<FrameAllocator>>;

    constexpr static v4l2_memory memory = V4L2_MEMORY_USERPTR;

    using base::base;

    explicit capture_userptr(const std::string& device, size_t buffers = 4)
        : base(device),
          m_allocator(buffers),
          m_num_buffers(buffers)
    {
    }

    FrameAllocator& allocator()
    {
        return m_allocator;
    }

    const FrameAllocator& allocator() const
    {
        return m_allocator;
    }

private:
    void check_caps(const v4l2_capability &cap)
    {
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw std::system_error({EINVAL, std::system_category()},
                                    fmt::format("{0} does not support streaming i/o", base::device()));
        }
    }

    void init_io()
    {
        const auto type = base::is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        v4l2_requestbuffers req {};

        req.count = m_num_buffers;
        req.type = type;
        req.memory = memory;

        priv::xioctl(base::fd(), VIDIOC_REQBUFS, &req);

        m_num_buffers = req.count;
        m_allocator.set_buffers_count(req.count);
        m_allocator.set_num_planes(base::m_format.num_planes);
        m_allocator.set_format(base::width(), base::height(), base::pixelformat());
        for (size_t i = 0; i < base::m_format.num_planes; ++i) {
            m_allocator.set_plane_size(i, base::m_format.plane_descr[i].sizeimage);
            //m_allocator.set_plane_size(i, base::m_format.plane_descr[i].bytesperline);
        }

        std::cout << fmt::format("REQBUFS: caps={0}", req.reserved[0]) << '\n';

        ///
        for (size_t n = 0; n < req.count; ++n) {
            v4l2_buffer buf {};
            v4l2_plane planes[VIDEO_MAX_PLANES] {};

            buf.type = type;
            buf.memory = memory;
            buf.index = n;

            if (base::is_mplane()) {
                buf.length = base::m_format.num_planes;
                buf.m.planes = planes;
            }

            priv::xioctl(base::fd(), VIDIOC_QUERYBUF, &buf);
            // Useful information after this call:
            //  buf.m.planes[i].length
            // or
            //  buf.length
            //
        }

    }

    void do_start()
    {
        auto type = base::is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        // Feed buffer intos V4L2 subsystem
        for (size_t i = 0; i < m_num_buffers; ++i) {
            v4l2_buffer buf {};
            v4l2_plane planes[format_descriptor::max_planes] {};

            buf.type = type;
            buf.memory = memory;
            buf.index = i;

            struct memory mem[format_descriptor::max_planes];
            m_allocator.aquire(i, mem, base::m_format.num_planes);

            if (base::is_mplane()) {
                buf.length = base::m_format.num_planes;
                buf.m.planes = planes;

                for (size_t plane = 0; plane < base::m_format.num_planes; ++plane) {
                    buf.m.planes[plane].length = mem[plane].length;
                    buf.m.planes[plane].m.userptr = (unsigned long)mem[plane].start;
                }
            } else {
                buf.length = mem[0].length;
                buf.m.userptr = reinterpret_cast<unsigned long>(mem[0].start);
            }

            priv::xioctl(base::fd(), VIDIOC_QBUF, &buf);
        }

        // Start
        priv::xioctl(base::fd(), VIDIOC_STREAMON, &type);
    }

    bool do_read(const on_read_proc_t& on_read)
    {
        const auto type = base::is_mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        v4l2_buffer buf;
        v4l2_plane planes[format_descriptor::max_planes] = {};

        buf.type = type;
        buf.memory = memory;

        if (base::is_mplane()) {
            buf.length = base::m_format.num_planes;
            buf.m.planes = planes;
        }

        if (!priv::xioctl(base::fd(), VIDIOC_DQBUF, &buf))
            return false;

        struct memory mem[format_descriptor::max_planes];

        if (base::is_mplane()) {
            for (size_t i = 0; i < buf.length; ++i) {
                mem[i].bytesused = buf.m.planes[i].bytesused;
                mem[i].start = reinterpret_cast<void*>(buf.m.planes[i].m.userptr);
            }
        } else {
            mem[0].bytesused = buf.bytesused;
            mem[0].start = reinterpret_cast<void*>(buf.m.userptr);
        }

        // frame callback
        if (on_read) {
            on_read(mem, base::m_format.num_planes);
        }

        // Release and aquire buffer again
        m_allocator.release(buf.index, mem, base::m_format.num_planes);
        m_allocator.aquire(buf.index, mem, base::m_format.num_planes);

        if (base::is_mplane()) {
            memset(planes, 0, sizeof(planes));
            buf.length = base::m_format.num_planes;
            buf.m.planes = planes;

            for (size_t i = 0; i < base::m_format.num_planes; ++i) {
                buf.m.planes[i].length = mem[i].length;
                buf.m.planes[i].m.userptr = reinterpret_cast<unsigned long>(mem[i].start);
            }

        } else {
            buf.length = mem[0].length;
            buf.m.userptr = reinterpret_cast<unsigned long>(mem[0].start);
        }

        priv::xioctl(base::fd(), VIDIOC_QBUF, &buf);

        return true;
    }


private:
    FrameAllocator m_allocator;
    size_t m_num_buffers;
};

} // namespace v4l2








template <typename T>
static void mainloop(T& dev, unsigned int frame_count)
{
    unsigned int count;

    count = frame_count;

    std::vector<uint8_t> buf;

    buf.resize(1920 * 1080 * 3 / 2);

    while (count-- > 0) {
        for (;;) {
            switch (dev.wait_frame(2s)) {
                case v4l2::frame_status::Interrupt:
                    continue;
                case v4l2::frame_status::Timeout:
                    return;
                default:
                    break;
            }

            auto st = dev.read([&buf](auto const& mem, auto num_mem){
                memcpy(buf.data(), mem[0].start, mem[0].bytesused);
            });
            if (st)
                break;
            /* EAGAIN - continue select loop. */
        }
    }
}



struct cma_frame_allocator
{
    constexpr static size_t single_buffer_size = 4096*2160*3/2;
    constexpr static size_t offsets[4] = {
        0 * single_buffer_size,
        1 * single_buffer_size,
        2 * single_buffer_size,
        3 * single_buffer_size
    };

    udmabuf dmabuf;
    uint8_t* ptr = nullptr;
    uint8_t* buf[4];

    explicit cma_frame_allocator(size_t buffers)
        : dmabuf("udmabuf@0x00")
    {
        assert(buffers <= 4);

        ptr = reinterpret_cast<uint8_t*>(dmabuf.get());

        buf[0] = ptr + 0 * single_buffer_size;
        buf[1] = ptr + 1 * single_buffer_size;
        buf[2] = ptr + 2 * single_buffer_size;
        buf[3] = ptr + 3 * single_buffer_size;
    }

    void set_buffers_count(size_t count)
    {

    }

    void set_num_planes(size_t num_planes)
    {
        plane_size.resize(num_planes);
    }

    void set_plane_size(size_t plane, size_t size)
    {
        plane_size[plane] = size;
    }

    void set_format(uint32_t width, uint32_t height, uint32_t pix_fmt)
    {

    }

    void aquire(size_t index, v4l2::memory *mem, size_t num_mem)
    {
        memset(mem, 0, num_mem*sizeof(*mem));
        assert(num_mem == 1);

        mem[0].start = buf[index];
        mem[0].length = plane_size[0];
    }

    void release(size_t index, v4l2::memory *mem, size_t num_mem)
    {
        // sync buffer
        dmabuf.sync_for_cpu(offsets[index], single_buffer_size);

        // now, access them!
    }

    std::vector<size_t> plane_size;
};



int main()
{
    //v4l2::capture_mmap dev("/dev/video0");
    v4l2::capture_userptr<cma_frame_allocator> dev("/dev/video0");
    //v4l2::capture_userptr dev("/dev/video0");

    dev.init();
    dev.start();

    mainloop(dev, 6400000);

}
