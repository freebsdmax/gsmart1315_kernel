#include <linux/module.h>
#include <mach/rpc_vbatt.h>
#include <mach/msm_rpcrouter.h>
#include <linux/err.h>

#define BATTERY_REGISTER_PROC	2
#define BATTERY_MODIFY_CLIENT_PROC	4
#define BATTERY_DEREGISTER_CLIENT_PROC	5
#define BATTERY_READ_MV_PROC	12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC	14

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1	0x00040001

#define BATT_RPC_TIMEOUT	5000

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

static const u32 battery_valid_rpc_vers[] = {
	BATTERY_RPC_VER_4_1,
	BATTERY_RPC_VER_2_1,
	BATTERY_RPC_VER_1_1,
};

struct msm_pm_vbatt_modify_client_arg {
	s32 handle;
	u32 vbatt_voltage;
	u32 voltage_direction;
	u32 cb_id;
	u32 cb_data;
};

struct msm_pm_vbatt_modify_client_ret {
	u32 result;
};

static int msm_pm_vbatt_modify_client_arg_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	int size = 0;
	unsigned int *req = (unsigned int *)buf;
	struct msm_pm_vbatt_modify_client_arg *arg =
		(struct msm_pm_vbatt_modify_client_arg *)data;

	*req = cpu_to_be32(arg->handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->vbatt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_pm_vbatt_modify_client_ret_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_modify_client_ret *buf_ptr =
		(struct msm_pm_vbatt_modify_client_ret *)buf;
	struct msm_pm_vbatt_modify_client_ret *data_ptr =
		(struct msm_pm_vbatt_modify_client_ret *)data;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

int msm_pm_vbatt_modify_client(struct msm_rpc_client *clnt,
	s32 handle,
	u32 desired_batt_voltage,
	u32 voltage_direction,
	u32 batt_cb_id,
	u32 cb_data)
{
	int rc = 0;
	struct msm_pm_vbatt_modify_client_arg arg;
	struct msm_pm_vbatt_modify_client_ret ret;

	arg.handle = handle;
	arg.vbatt_voltage = desired_batt_voltage;
	arg.voltage_direction = voltage_direction;
	arg.cb_id = batt_cb_id;
	arg.cb_data = cb_data;

	rc = msm_rpc_client_req(clnt,
		BATTERY_MODIFY_CLIENT_PROC,
		msm_pm_vbatt_modify_client_arg_func,
		&arg,
		msm_pm_vbatt_modify_client_ret_func,
		&ret,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (0 > rc) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
		       __func__);
		return rc;
	}

	if (BATTERY_MODIFICATION_SUCCESSFUL != ret.result) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, ret.result);
		return -EIO;
	}

	return rc;
}
EXPORT_SYMBOL(msm_pm_vbatt_modify_client);

struct msm_pm_vbatt_register_arg {
	u32 desired_batt_voltage;
	u32 voltage_direction;
	u32 batt_cb_id;
	u32 cb_data;
	u32 batt_error;
};

struct msm_pm_vbatt_register_ret {
	s32 batt_handle;
};

static int msm_pm_vbatt_register_arg_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	unsigned int *req = (unsigned int *)buf;
	int size = 0;
	struct msm_pm_vbatt_register_arg *arg =
		(struct msm_pm_vbatt_register_arg *)data;

	*req = cpu_to_be32(arg->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->batt_error);
	size += sizeof(u32);

	return size;
}

static int msm_pm_vbatt_register_ret_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_register_ret *data_ptr =
		(struct msm_pm_vbatt_register_ret *)data;
	struct msm_pm_vbatt_register_ret *buf_ptr =
		(struct msm_pm_vbatt_register_ret *)buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

int msm_pm_vbatt_register(struct msm_rpc_client *clnt,
	u32 desired_batt_voltage,
	u32 voltage_direction,
	u32 batt_cb_id,
	u32 cb_data,
	s32 *handle)
{
	struct msm_pm_vbatt_register_arg arg;
	struct msm_pm_vbatt_register_ret ret;
	int rc = 0;

	arg.desired_batt_voltage = desired_batt_voltage;
	arg.voltage_direction = voltage_direction;
	arg.batt_cb_id = batt_cb_id;
	arg.cb_data = cb_data;
	arg.batt_error = 1;

	rc = msm_rpc_client_req(clnt,
		BATTERY_REGISTER_PROC,
		msm_pm_vbatt_register_arg_func,
		&arg,
		msm_pm_vbatt_register_ret_func,
		&ret,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (0 > rc) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	*handle = ret.batt_handle;

	return rc;
}
EXPORT_SYMBOL(msm_pm_vbatt_register);

struct msm_pm_vbatt_deregister_client_arg {
	s32 handle;
};

struct msm_pm_vbatt_deregister_client_ret {
	u32 batt_error;
};

static int msm_pm_vbatt_deregister_client_arg_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_deregister_client_arg *arg =
		(struct msm_pm_vbatt_deregister_client_arg *)data;
	unsigned int *req = (unsigned int *)buf;
	int size = 0;

	*req = cpu_to_be32(arg->handle);
	size += sizeof(u32);

	return size;
}

static int msm_pm_vbatt_deregister_client_ret_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_deregister_client_ret *data_ptr =
		(struct msm_pm_vbatt_deregister_client_ret *)data;
	struct msm_pm_vbatt_deregister_client_ret *buf_ptr =
		(struct msm_pm_vbatt_deregister_client_ret *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

int msm_pm_vbatt_deregister_client(struct msm_rpc_client *clnt, int handle)
{
	int rc = 0;
	struct msm_pm_vbatt_deregister_client_arg arg;
	struct msm_pm_vbatt_deregister_client_ret ret;

	arg.handle = handle;

	rc = msm_rpc_client_req(clnt,
		BATTERY_DEREGISTER_CLIENT_PROC,
		msm_pm_vbatt_deregister_client_arg_func,
		&arg,
		msm_pm_vbatt_deregister_client_ret_func,
		&ret,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (0 > rc) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (BATTERY_DEREGISTRATION_SUCCESSFUL != ret.batt_error) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, ret.batt_error, handle);
		return -EIO;
	}

	return rc;
}
EXPORT_SYMBOL(msm_pm_vbatt_deregister_client);

struct msm_pm_vbatt_enable_disable_filter_arg {
	s32 handle;
	u32 enable_or_disable_filter;
	u32 filter;
};

struct msm_pm_vbatt_enable_disable_filter_ret {
	u32 result;
};

static int msm_pm_vbatt_enable_disable_filter_arg_func(
	struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	u32 *req = (unsigned int *)buf;
	int size = 0;
	struct msm_pm_vbatt_enable_disable_filter_arg *arg =
		(struct msm_pm_vbatt_enable_disable_filter_arg *)data;

	*req = cpu_to_be32(arg->handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->enable_or_disable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(arg->filter);
	size += sizeof(u32);

	return size;
}

static int msm_pm_vbatt_enable_disable_filter_ret_func(
	struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_enable_disable_filter_ret *buf_ptr =
		(struct msm_pm_vbatt_enable_disable_filter_ret *)buf;
	struct msm_pm_vbatt_enable_disable_filter_ret *data_ptr =
		(struct msm_pm_vbatt_enable_disable_filter_ret *)data;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

int msm_pm_vbatt_enable_disable_filter(struct msm_rpc_client *clnt,
	s32 handle,
	s32 enable,
	u32 filter)
{
	int rc = 0;
	struct msm_pm_vbatt_enable_disable_filter_arg arg;
	struct msm_pm_vbatt_enable_disable_filter_ret ret;

	arg.handle = handle;
	arg.enable_or_disable_filter = enable;
	arg.filter = filter;

	rc = msm_rpc_client_req(clnt,
		BATTERY_ENABLE_DISABLE_FILTER_PROC,
		msm_pm_vbatt_enable_disable_filter_arg_func,
		&arg,
		msm_pm_vbatt_enable_disable_filter_ret_func,
		&ret,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (0 > rc) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (BATTERY_DEREGISTRATION_SUCCESSFUL != ret.result) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, ret.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);

	return rc;
}
EXPORT_SYMBOL(msm_pm_vbatt_enable_disable_filter);

struct msm_pm_vbatt_read_mv_ret {
	u32 batt_voltage;
};

static int msm_pm_vbatt_read_mv_ret_func(struct msm_rpc_client *clnt,
	void *buf,
	void *data)
{
	struct msm_pm_vbatt_read_mv_ret *buf_ptr, *data_ptr;

	buf_ptr = (struct msm_pm_vbatt_read_mv_ret *)buf;
	data_ptr = (struct msm_pm_vbatt_read_mv_ret *)data;

	data_ptr->batt_voltage = be32_to_cpu(buf_ptr->batt_voltage);

	return 0;
}

u32 msm_pm_vbatt_read_mv(struct msm_rpc_client *clnt)
{
	int rc = 0;
	struct msm_pm_vbatt_read_mv_ret ret;

	rc = msm_rpc_client_req(clnt,
		BATTERY_READ_MV_PROC,
		NULL,
		NULL,
		msm_pm_vbatt_read_mv_ret_func,
		&ret,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (0 > rc) {
		pr_err("%s: FAIL: vbatt get volt. rc=%d\n", __func__, rc);
		return 0;
	}

	return ret.batt_voltage;
}
EXPORT_SYMBOL(msm_pm_vbatt_read_mv);

struct msm_rpc_client *msm_pm_vbatt_init_client(
	char *name,
	u32 create_thread,
	int (*cb_func)(struct msm_rpc_client *, void *, int),
	u32 *version)
{
	int i;
	struct msm_rpc_client *clnt;

	for (i = 0; i < ARRAY_SIZE(battery_valid_rpc_vers); i++) {
		clnt = msm_rpc_register_client(name,
			BATTERY_RPC_PROG,
			battery_valid_rpc_vers[i],
			create_thread,
			cb_func);
		if (NULL == clnt) {
			pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
			       __func__);
			return NULL;
		} else if (IS_ERR(clnt))
			continue;
		else {
			if (NULL != version)
				*version = battery_valid_rpc_vers[i];
			break;
		}
	}

	if (IS_ERR(clnt))
		clnt = NULL;

	return clnt;
}
EXPORT_SYMBOL(msm_pm_vbatt_init_client);
