#ifndef __RPC_VBATT_H__
#define __RPC_VBATT_H__

#include <mach/msm_rpcrouter.h>

int msm_pm_vbatt_modify_client(struct msm_rpc_client *, s32, u32, u32, u32,
							   u32);

int msm_pm_vbatt_register(struct msm_rpc_client *, u32, u32, u32, u32, s32 *);

int msm_pm_vbatt_deregister_client(struct msm_rpc_client *, s32);

int msm_pm_vbatt_enable_disable_filter(struct msm_rpc_client *, s32, s32, u32);

u32 msm_pm_vbatt_read_mv(struct msm_rpc_client *);

struct msm_rpc_client *msm_pm_vbatt_init_client(char *, u32,
	int (*)(struct msm_rpc_client *, void *, int), u32 *);

#endif
