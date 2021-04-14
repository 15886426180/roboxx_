#pragma once

#include <new>
#include <memory>
#include <atomic>
#include <cstdint>

#ifdef __cpp_lib_hardware_interference_size
    using std::hardware_constructive_interference_size;
#else
    static constexpr std::size_t hardware_constructive_interference_size{2 * sizeof(std::max_align_t)};
#endif

 struct ticket_mutex
    {
    public:
        inline void lock(void)
        {
            auto const ticket_{m_in.fetch_add(1, std::memory_order::acquire)};
            while (true)
            {
                auto const now_{m_out.load(std::memory_order::acquire)};
                if (now_ == ticket_)
                    return;
                m_out.wait(now_, std::memory_order::relaxed);
            }
        }

        inline void unlock(void)
        {
            m_out.fetch_add(1, std::memory_order::release);
            m_out.notify_all();
        }

    private:
        alignas(hardware_constructive_interference_size) std::atomic<std::size_t> m_in{ATOMIC_VAR_INIT(0)};
        alignas(hardware_constructive_interference_size) std::atomic<std::size_t> m_out{ATOMIC_VAR_INIT(0)};
    };
