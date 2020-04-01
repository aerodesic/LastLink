/*
 * vfs_lastlink.c
 *
 * Implement the vfs support for the lastlink socket layer.
 */

#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/errno.h>
#include <sys/lock.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_attr.h"
#include "lsocket_internal.h"
#include "linklayer.h"
#include "sdkconfig.h"

_Static_assert(MAX_FDS >= LAST_LASTLINK_FD, "MAX_FDS < LAST_LASTLINK_FD");

static void ls_stop_socket_select(void *sem)
{
    os_release_semaphore((os_semaphore_t) sem);
}

static void ls_stop_socket_select_isr(void *sem, BaseType_t *woken)
{
    bool awakened;
    os_release_semaphore_from_isr((os_semaphore_t) sem, &awakened);
    *woken = awakened ? pdTRUE : pdFALSE;
}

static void *ls_get_socket_select_semaphore(void)
{
    return (void *) os_thread_sem_get();
}

static int ls_fcntl_r_wrapper(int fd, int cmd, int arg)
{
    return ls_fcntl(fd, cmd, arg);
}

static int ls_ioctl_r_wrapper(int fd, int cmd, va_list args)
{
    return ls_ioctl(fd, cmd, va_arg(args, void *));
}

void esp_vfs_ls_sockets_register(void)
{
    esp_vfs_t vfs = {
        .flags = ESP_VFS_FLAG_DEFAULT,
        .write = &ls_write,
        .open = NULL,
        .fstat = NULL,
        .close = &ls_close,
        .read = &ls_read,
        .fcntl = &ls_fcntl_r_wrapper,
        .ioctl = &ls_ioctl_r_wrapper,
        .socket_select = &ls_select,
        .get_socket_select_semaphore = &ls_get_socket_select_semaphore,
        .stop_socket_select = &ls_stop_socket_select,
        .stop_socket_select_isr = &ls_stop_socket_select_isr,
    };

    ESP_ERROR_CHECK(esp_vfs_register_fd_range(&vfs, NULL, FIRST_LASTLINK_FD, LAST_LASTLINK_FD));
}
