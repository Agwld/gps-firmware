/**
 * @file    syscalls_errno.c
 * @brief   Minimal __errno() to satisfy libm's internal error-reporting
 *          references.
 *
 * This toolchain's prebuilt libm.a (nano.specs, thumb/v7e-m+fp/hard)
 * ships an __errno object the linker rejects as an unsupported ARM/
 * Thumb interworking relocation - a toolchain packaging defect, not
 * anything under this project's control (confirmed by inspecting
 * libc.a: the symbol exists and is referenced correctly, but its
 * encoded destination mode is wrong). Nothing in this firmware ever
 * reads errno - there's no syscall layer to set it in the first place -
 * so a trivial single, non-reentrant instance satisfies every internal
 * libm reference without depending on the broken prebuilt one.
 */

static int s_errno_storage;

int *
__errno(void)
{
    return &s_errno_storage;
}
