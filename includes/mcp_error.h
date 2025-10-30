#ifndef MCPERROR_H
#define MCPERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <errno.h>

typedef struct mcp_dev_base mcp_dev_base_t;

typedef enum {
    MCP_OK          =  0,
    MCP_EPARAM      = -1,   
    MCP_ENOTSUP     = -2,
    MCP_ENODEV      = -3,
    MCP_EBUS        = -4,
    MCP_ETIMEOUT    = -5,
    MCP_EIO         = -6,
    MCP_ESTATE      = -7,
    MCP_ECONFIG     = -8,
    MCP_ENOMEM      = -9,
    MCP_ECRC        = -10,
    MCP_EAGAIN_     = -11
} mcp_err_t;

typedef struct {
    int         code;
    int         sys_errno;
    const char* file;
    const char* func;
    int         line;
    char        msg[128];
} mcp_error_t;

/*
 * @brief Returns the symbolic name of the given error code.
 *
 * @param code          Error code (from mcp_err_t).
 * 
 * @return const char*  Symbolic error name 
 */
const char* mcp_get_err_name(int code);

/*
 * @brief Returns a human-readable error message for the given error code.
 *
 * @param code          Error code (from mcp_err_t).
 * 
 * @return const char*  Error message
 */
const char* mcp_strerror(int code);

/*
 * @brief Maps the current errno to an appropriate mcp_err_t code.
 *
 * @return int  Mapped mcp_err_t code
 */
int mcp_map_errno(void);

/*
 * @brief Sets the last error information in the device handle.
 *
 * @param base          Pointer to the MCP23 device handle.
 * @param code          Error code (from mcp_err_t).
 * @param sys_errno     System errno from failed syscall (or 0).
 * @param file          Source file name (__FILE__).
 * @param func          Function name (__func__).
 * @param line          Line number (__LINE__).
 */
void mcp_set_error(mcp_dev_base_t* base, int code, int sys_errno,
                   const char* file, const char* func, int line);

/*
 * @brief Retrieves the last error information from the device handle.
 *
 * @param base          Pointer to the MCP23 device handle.
 * 
 * @return mcp_error_t  Last error information
 */
mcp_error_t mcp_last_error(const mcp_dev_base_t* base);

static inline int mcp_fail_errno(mcp_dev_base_t* base, int code,
                                 const char* file, const char* func, int line) {
    mcp_set_error(base, code, errno, file, func, line);
    return code;
}

#define MCP_FAIL(dev, code) \
    mcp_fail_errno(&(dev)->base, (code), __FILE__, __func__, __LINE__)

#ifdef __cplusplus
}
#endif

#endif /* MCPERROR_H */