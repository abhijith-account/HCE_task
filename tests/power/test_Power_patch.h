// test_Power_patch.h
// Runtime code-patching helpers (Linux 64-bit only): find a symbol in our own ELF image and
// overwrite a function with "return false". Used to reach branches that cannot otherwise
// be forced on the host build. All functions are inline so the header is safe in many TUs.
#pragma once

#include "test_power_common.h"

#if defined(__linux__) && defined(__LP64__) && (defined(__x86_64__) || defined(__aarch64__))
#define PM_TEST_CAN_PATCH_CODE 1
#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <string>

namespace pm_test_patch {

inline int exe_base_cb(struct dl_phdr_info* info, size_t, void* data) {
    if (info->dlpi_name == nullptr || info->dlpi_name[0] == '\0') {
        *static_cast<uintptr_t*>(data) = static_cast<uintptr_t>(info->dlpi_addr);
        return 1;
    }
    return 0;
}

// Finds a (possibly internal-linkage) function/object symbol in our own executable
// by reading the ELF symbol table in-process (no dependency on `nm`).
// Works for PIE (load bias != 0) and non-PIE (load bias == 0) binaries.
inline uint8_t* find_own_symbol(const char* needle, unsigned char sym_type, size_t min_size) {
    FILE* f = std::fopen("/proc/self/exe", "rb");
    if (!f) return nullptr;

    std::fseek(f, 0, SEEK_END);
    long fsz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (fsz <= 0) { std::fclose(f); return nullptr; }

    std::string img(static_cast<size_t>(fsz), '\0');
    size_t rd = std::fread(&img[0], 1, img.size(), f);
    std::fclose(f);
    if (rd != img.size() || img.size() < sizeof(Elf64_Ehdr)) return nullptr;

    const auto* eh = reinterpret_cast<const Elf64_Ehdr*>(img.data());
    if (std::memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0) return nullptr;
    if (eh->e_shoff == 0 || eh->e_shoff + eh->e_shnum * sizeof(Elf64_Shdr) > img.size()) return nullptr;

    const auto* sh = reinterpret_cast<const Elf64_Shdr*>(img.data() + eh->e_shoff);
    uintptr_t base = 0;
    dl_iterate_phdr(exe_base_cb, &base);

    for (unsigned i = 0; i < eh->e_shnum; ++i) {
        if (sh[i].sh_type != SHT_SYMTAB) continue;
        if (sh[i].sh_link >= eh->e_shnum) continue;
        const Elf64_Shdr& str = sh[sh[i].sh_link];
        if (sh[i].sh_offset + sh[i].sh_size > img.size()) continue;
        if (str.sh_offset + str.sh_size > img.size()) continue;

        const auto* syms = reinterpret_cast<const Elf64_Sym*>(img.data() + sh[i].sh_offset);
        const char* names = img.data() + str.sh_offset;
        size_t count = sh[i].sh_size / sizeof(Elf64_Sym);

        for (size_t s = 0; s < count; ++s) {
            if (ELF64_ST_TYPE(syms[s].st_info) != sym_type) continue;
            if (syms[s].st_value == 0 || syms[s].st_size < min_size) continue;
            if (syms[s].st_name >= str.sh_size) continue;
            if (std::strstr(names + syms[s].st_name, needle) == nullptr) continue;
            return reinterpret_cast<uint8_t*>(base + syms[s].st_value);
        }
    }
    return nullptr;
}

inline uint8_t* find_own_function(const char* needle) {
    return find_own_symbol(needle, STT_FUNC, 8);
}

// Overwrites the start of `fn` with "return false" and returns the original
// bytes so the caller can restore them.
inline bool patch_return_false(uint8_t* fn, uint8_t (&orig)[8]) {
    const uintptr_t page = static_cast<uintptr_t>(sysconf(_SC_PAGESIZE));
    const uintptr_t start = reinterpret_cast<uintptr_t>(fn) & ~(page - 1);
    const uintptr_t end   = (reinterpret_cast<uintptr_t>(fn) + 8 + page - 1) & ~(page - 1);
    if (mprotect(reinterpret_cast<void*>(start), end - start,
                 PROT_READ | PROT_WRITE | PROT_EXEC) != 0) {
        return false;
    }
    std::memcpy(orig, fn, 8);
#if defined(__x86_64__)
    fn[0] = 0x31; fn[1] = 0xC0;          // xor eax, eax
    fn[2] = 0xC3;                        // ret
#elif defined(__aarch64__)
    const uint32_t code[2] = {0x52800000u, 0xD65F03C0u};  // mov w0,#0 ; ret
    std::memcpy(fn, code, sizeof(code));
#endif
    __builtin___clear_cache(reinterpret_cast<char*>(fn), reinterpret_cast<char*>(fn + 8));
    return true;
}

inline void restore(uint8_t* fn, const uint8_t (&orig)[8]) {
    const uintptr_t page = static_cast<uintptr_t>(sysconf(_SC_PAGESIZE));
    const uintptr_t start = reinterpret_cast<uintptr_t>(fn) & ~(page - 1);
    const uintptr_t end   = (reinterpret_cast<uintptr_t>(fn) + 8 + page - 1) & ~(page - 1);
    std::memcpy(fn, orig, 8);
    __builtin___clear_cache(reinterpret_cast<char*>(fn), reinterpret_cast<char*>(fn + 8));
    mprotect(reinterpret_cast<void*>(start), end - start, PROT_READ | PROT_EXEC);
}

}  // namespace pm_test_patch
#endif
