#include "../includes/mcp_error.h"

#include <stdio.h>
#include <string.h>
#include "mcp23.h"

const char* mcp_get_err_name(int code) {
    switch (code) {
        case MCP_OK:       return "MCP_OK";
        case MCP_EPARAM:   return "MCP_EPARAM";
        case MCP_ENOTSUP:  return "MCP_ENOTSUP";
        case MCP_ENODEV:   return "MCP_ENODEV";
        case MCP_EBUS:     return "MCP_EBUS";
        case MCP_ETIMEOUT: return "MCP_ETIMEOUT";
        case MCP_EIO:      return "MCP_EIO";
        case MCP_ESTATE:   return "MCP_ESTATE";
        case MCP_ECONFIG:  return "MCP_ECONFIG";
        case MCP_ENOMEM:   return "MCP_ENOMEM";
        case MCP_ECRC:     return "MCP_ECRC";
        case MCP_EAGAIN_:  return "MCP_EAGAIN";
        default:           return "MCP_EUNKNOWN";
    }
}

const char* mcp_strerror(int code) {
    switch (code) {
        case MCP_OK:       return "Success";
        case MCP_EPARAM:   return "Invalid parameter";
        case MCP_ENOTSUP:  return "Operation not supported";
        case MCP_ENODEV:   return "Device not present or not responding";
        case MCP_EBUS:     return "Bus error (SPI/I2C transaction failed)";
        case MCP_ETIMEOUT: return "Operation timed out";
        case MCP_EIO:      return "I/O error";
        case MCP_ESTATE:   return "Invalid device state";
        case MCP_ECONFIG:  return "Invalid or inconsistent configuration";
        case MCP_ENOMEM:   return "Out of memory";
        case MCP_ECRC:     return "Data integrity error";
        case MCP_EAGAIN_:  return "Resource temporarily unavailable, try again";
        default:           return "Unknown error";
    }
}

int mcp_map_errno(void) {
    switch (errno) {
        case 0:           return MCP_OK;
        case EINVAL:      return MCP_ECONFIG;
        case ENOTDIR:     return MCP_ECONFIG;
        case EISDIR:      return MCP_ECONFIG;
        case ENOSYS:      return MCP_ENOTSUP;
        case ENOTTY:      return MCP_ENOTSUP;
        case EOPNOTSUPP:  return MCP_ENOTSUP;
        case ENODEV:      return MCP_ENODEV;
        case ENXIO:       return MCP_ENODEV;
        case ETIMEDOUT:   return MCP_ETIMEOUT;
        case EAGAIN:      return MCP_EAGAIN_;
        case EINTR:       return MCP_EAGAIN_;
        case EBUSY:       return MCP_EAGAIN_;
        case EIO:         return MCP_EIO;
        case EFAULT:      return MCP_EIO;
        case ENOMEM:      return MCP_ENOMEM;
        case EACCES:      return MCP_ESTATE;
        case EPERM:       return MCP_ESTATE;
        case EBADF:       return MCP_ESTATE;
        case EPROTO:      return MCP_EBUS;
        default:          return MCP_EBUS;
    }
}

void mcp_set_error(mcp_dev_base_t* base, int code, int sys_errno,
                   const char* file, const char* func, int line) {

    if (!base) {
        return;
    }

    base->last_error.code = code;
    base->last_error.sys_errno = sys_errno;
    base->last_error.file = file;
    base->last_error.func = func;
    base->last_error.line = line;
}

mcp_error_t mcp_last_error(const mcp_dev_base_t* base) {
    mcp_error_t e = {0};

    if (!base) {
        return e;
    }
    return base->last_error;
}