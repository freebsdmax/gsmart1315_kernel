#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

static struct i2c_client *hack_client = NULL;
static const struct i2c_device_id *hack_id = NULL;

static int found = 0;

int hack_i2c_add_driver(struct i2c_driver *driver)
{
	int rc;

	if ( found )
		return -ENODEV;

	rc = driver->probe(hack_client, hack_id);

	printk("<0>--CAMERA-- Hack hack_i2c_add_driver %i\n", rc);

	if ( rc == 0 )
		found = 1;

	return rc;
}

static int msm_qrd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	hack_client = client;
	hack_id = id;

	return 0;
}

static int msm_qrd_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id msm_qrd_i2c_id[] = {
	{"msm_qrd", 0},{}
};

static struct i2c_driver msm_qrd_i2c_driver = {
	.id_table = msm_qrd_i2c_id,
	.probe  = msm_qrd_i2c_probe,
	.remove = msm_qrd_i2c_remove,
	.driver = {
		.name = "msm_qrd",
	},
};

static int __init msm_qrd_init(void)
{
	int rc;

	rc = i2c_add_driver(&msm_qrd_i2c_driver);

	printk("<0>--CAMERA-- i2c_add_driver %i\n", rc);

	return rc;
}

module_init(msm_qrd_init);

