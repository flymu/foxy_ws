// dds_bomb_guard.cpp — LD_PRELOAD 全局operator new护栏:
// 拒绝任何单个 >4GB 的分配请求(实测毒包声称100~1065GB), 合法业务单个分配远低于此。
// FastDDS 2.1.4/fastcdr 1.0.13 的 XCDR2 TypeInformation 解析炸弹由此彻底失效。
// 被拒分配抛出 std::bad_alloc, 由 fastrtps 自身的 catch 正常吞掉, 发现流程不受影响。
#include <cstdio>
#include <cstdlib>
#include <new>

static const size_t CAP = (size_t)4 << 30; // 4 GiB

static inline void* guard_alloc(size_t sz, bool nothrow) noexcept
{
    if (sz > CAP)
    {
        static int logged = 0;
        if (logged < 30)
        {
            fprintf(stderr, "[dds_bomb_guard] 拒绝 %.1f GB 的单笔分配请求 (FastDDS XCDR2 毒包防御)\n",
                    (double)sz / 1073741824.0);
            logged++;
        }
        return nullptr; // 调用方(operator new)负责抛bad_alloc
    }
    return std::malloc(sz ? sz : 1);
}

void* operator new(size_t sz)
{
    void* p = guard_alloc(sz, false);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new[](size_t sz)
{
    void* p = guard_alloc(sz, false);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new(size_t sz, const std::nothrow_t&) noexcept
{
    return guard_alloc(sz, true);
}

void* operator new[](size_t sz, const std::nothrow_t&) noexcept
{
    return guard_alloc(sz, true);
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
void operator delete[](void* p, size_t) noexcept { std::free(p); }
