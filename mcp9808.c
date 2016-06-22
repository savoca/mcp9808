/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Copyright (c) 2015, Alex Deddo <adeddo27@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/acpi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "mcp9808"

#define MCP_REG_CFG_SHDN 0x100
#define MCP_REG_CFG_CLCK 0x80
#define MCP_REG_CFG_WLCK 0x40
#define MCP_REG_CFG_IRQC 0x20
#define MCP_REG_CFG_ASTS 0x10
#define MCP_REG_CFG_ACTL 0x8
#define MCP_REG_CFG_ASEL 0x4
#define MCP_REG_CFG_APOL 0x2
#define MCP_REG_CFG_AMOD 0x1

#define MCP_REG_CONFIG 0x01
#define MCP_REG_UPPER  0x02
#define MCP_REG_LOWER  0x03
#define MCP_REG_CRIT   0x04
#define MCP_REG_TEMP   0x05
#define MCP_REG_VENDOR 0x06
#define MCP_REG_DEVICE 0x07
#define MCP_REG_RES    0x08

#define MCP_VENDOR 0x0054
#define MCP_DEVICE 0x0400

enum {
	MCP_RES_MIN = 0,
	MCP_RES_LOW,
	MCP_RES_HGH,
	MCP_RES_MAX,
};

struct mcp9808_data {
	dev_t chr_dev;
	struct cdev chr_cdev;
	struct class *chr_class;
	struct device *chr_device;
	struct i2c_client *client;
	uint8_t chip_state;
};

static int mcp9808_i2c_write(struct i2c_client *client,
	uint8_t reg, uint8_t *data, size_t len)
{
	uint8_t buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		pr_err("failed to write to chip\n");
		return -EIO;
	}

	return 0;
}

static int mcp9808_i2c_read(struct i2c_client *client,
	uint8_t reg, uint8_t *data, size_t len)
{
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		pr_err("failed to read from chip\n");
		return -EIO;
	}

	return 0;
}

static int mcp9808_verify_chip(struct mcp9808_data *mcpd)
{
	uint8_t buf[2];
	uint16_t response;

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_VENDOR, buf, 2))
		return -EIO;

	response = (buf[0] << 8) | buf[1];
	if (response != MCP_VENDOR) {
		pr_err("invalid vendor response: 0x%04x\n", response);
		return -EINVAL;
	}

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_DEVICE, buf, 2))
		return -EIO;

	response = (buf[0] << 8) | buf[1];
	if (response != MCP_DEVICE) {
		pr_err("invalid device response: 0x%04x\n", response);
		return -EINVAL;
	}

	return 0;
}

static int mcp9808_get_chip_state(struct mcp9808_data *mcpd, uint8_t *state)
{
	uint8_t buf[2];
	uint16_t config;

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_CONFIG, buf, 2))
		return -EIO;

	config = (buf[0] << 8) | buf[1];

	*state = !(config >> 8) & 1;

	return 0;
}

static int mcp9808_set_chip_state(struct mcp9808_data *mcpd, int state)
{
	uint8_t buf[2];
	uint16_t config;

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_CONFIG, buf, 2))
		return -EIO;

	config = (buf[0] << 8) | buf[1];

	if (state == !((config >> 8) & 1)) {
		pr_err("chip already in requested state\n");
		return -EINVAL;
	}

	if (state)
		config &= ~MCP_REG_CFG_SHDN;
	else
		config |= MCP_REG_CFG_SHDN;

	buf[0] = (config & 0xff00) >> 8;
	buf[1] = (config & 0xff);

	if (mcp9808_i2c_write(mcpd->client, MCP_REG_CONFIG, buf, 2))
		return -EIO;

	mcpd->chip_state = state;
	return 0;
}

static int mcp9808_open(struct inode *inode, struct file *file)
{
	struct mcp9808_data *mcpd = container_of(
		inode->i_cdev, struct mcp9808_data, chr_cdev);

	if (!mcpd)
		return -ENODEV;

	file->private_data = mcpd;

	return 0;
}

static ssize_t mcp9808_read(struct file *file, char __user *buff,
	size_t count, loff_t *pos)
{
	int len = 0;
	uint8_t buf[2];
	struct mcp9808_data *mcpd = file->private_data;

	if (!mcpd)
		return -ENODEV;

	if (!mcpd->chip_state)
		return -EPERM;

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_TEMP, buf, 2))
		return -EIO;

	len = sizeof(buf);

	if (copy_to_user(buff, buf, len))
		return -EFAULT;

	return len;
}

static const struct file_operations mcp9808_fops = {
	.owner = THIS_MODULE,
	.open = mcp9808_open,
	.read = mcp9808_read,
	.llseek = no_llseek,
};

static ssize_t mcp9808_chip_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mcp9808_data *mcpd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mcpd->chip_state);
}

static ssize_t mcp9808_chip_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int r, state;
	struct mcp9808_data *mcpd = dev_get_drvdata(dev);

	r = kstrtoint(buf, 10, &state);
	if (r || state < 0 || state > 1 ||
			state == mcpd->chip_state)
		return -EINVAL;

	if (mcp9808_set_chip_state(mcpd, state))
		return -EIO;

	return count;
}

/* Resolution bits
 * MCP_RES_MIN = +0.5C (30ms conv)
 * MCP_RES_LOW = +0.25C (65ms conv)
 * MCP_RES_HGH = +0.125C (130ms conv)
 * MCP_RES_MAX = +0.0625C (250ms conv)
 */
static ssize_t mcp9808_res_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t res;
	struct mcp9808_data *mcpd = dev_get_drvdata(dev);

	if (mcp9808_i2c_read(mcpd->client, MCP_REG_RES, &res, 1))
		return -EIO;

	return scnprintf(buf, PAGE_SIZE, "%d\n", res);
}

static ssize_t mcp9808_res_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int r, data;
	struct mcp9808_data *mcpd = dev_get_drvdata(dev);
	uint8_t res;

	r = kstrtoint(buf, 10, &data);
	if (r || data < MCP_RES_MIN || data > MCP_RES_MAX)
		return -EINVAL;

	res = data & 0xff;

	if (mcp9808_i2c_write(mcpd->client, MCP_REG_RES, &res, 1))
		return -EIO;

	return count;
}

static struct device_attribute attrs[] = {
	__ATTR(chip_state, S_IWUSR | S_IRUGO, mcp9808_chip_state_show,
		mcp9808_chip_state_store),
	__ATTR(res, S_IWUSR | S_IRUGO, mcp9808_res_show,
		mcp9808_res_store),
};

static int mcp9808_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id)
{
	int ret = 0;
	uint8_t attr_count;
	struct mcp9808_data *mcpd;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c check functionality failed\n");
		return -ENODEV;
	}

	mcpd = kzalloc(sizeof(*mcpd), GFP_KERNEL);
	if (!mcpd) {
		pr_err("failed to alloc mem for mcp9808_data\n");
		return -ENOMEM;
	}

	mcpd->client = client;
	i2c_set_clientdata(client, mcpd);

	ret = mcp9808_verify_chip(mcpd);
	if (ret) {
		pr_err("failed to verify chip: %d\n", ret);
		goto err_free;
	}

	ret = mcp9808_get_chip_state(mcpd, &mcpd->chip_state);
	if (ret) {
		pr_err("failed to get chip state: %d\n", ret);
		goto err_free;
	}

	ret = alloc_chrdev_region(&mcpd->chr_dev, 0, 1, DRIVER_NAME);
	if (ret < 0) {
		pr_err("failed to alloc chrdev region\n");
		goto err_free;
	}

	cdev_init(&mcpd->chr_cdev, &mcp9808_fops);
	mcpd->chr_cdev.owner = THIS_MODULE;

	ret = cdev_add(&mcpd->chr_cdev, mcpd->chr_dev, 1);
	if (ret) {
		pr_err("failed to create cdev: %d\n", ret);
		goto err_unregister;
	}

	mcpd->chr_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(mcpd->chr_class)) {
		pr_err("failed to create class\n");
		ret = PTR_ERR(mcpd->chr_class);
		goto err_cdev;
	}

	mcpd->chr_device = device_create(mcpd->chr_class, NULL,
		mcpd->chr_dev, NULL, DRIVER_NAME);
	if (IS_ERR(mcpd->chr_device)) {
		pr_err("failed to create device\n");
		ret = PTR_ERR(mcpd->chr_device);
		goto err_class;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		ret = sysfs_create_file(&client->dev.kobj,
			&attrs[attr_count].attr);
		if (ret < 0) {
			pr_err("failed to create sysfs entries\n");
			goto err_sysfs;
		}
	}

	pr_info("registered chip\n");

	return 0;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&client->dev.kobj,
			&attrs[attr_count].attr);
	}
	device_destroy(mcpd->chr_class, mcpd->chr_dev);
err_class:
	class_destroy(mcpd->chr_class);
err_cdev:
	cdev_del(&mcpd->chr_cdev);
err_unregister:
	unregister_chrdev_region(mcpd->chr_dev, 1);
err_free:
	kfree(mcpd);
	return ret;
}

static int mcp9808_remove(struct i2c_client *client)
{
	uint8_t attr_count;
	struct mcp9808_data *mcpd = i2c_get_clientdata(client);

	for (attr_count = ARRAY_SIZE(attrs); attr_count > 0;) {
		sysfs_remove_file(&client->dev.kobj,
			&attrs[--attr_count].attr);
	}

	device_destroy(mcpd->chr_class, mcpd->chr_dev);
	class_destroy(mcpd->chr_class);
	cdev_del(&mcpd->chr_cdev);
	unregister_chrdev_region(mcpd->chr_dev, 1);

	kfree(mcpd);

	pr_info("unregistered chip\n");

	return 0;
}

static const struct acpi_device_id mcp9808_acpi_match[] = {
	{ "MCP9808", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, mcp9808_acpi_match);

static struct i2c_device_id mcp9808_id_table[] = {
	{ "mcp9808", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mcp9808_id_table);

static struct of_device_id mcp9808_match_table[] = {
	{ .compatible = "microchip,mcp9808", },
	{},
};

static struct i2c_driver mcp9808_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mcp9808_match_table,
		.acpi_match_table = ACPI_PTR(mcp9808_acpi_match),
	},
	.probe = mcp9808_probe,
	.remove = mcp9808_remove,
	.id_table = mcp9808_id_table,
};

static int __init mcp9808_init(void)
{
	if (i2c_add_driver(&mcp9808_driver))
		return -ENODEV;

	return 0;
}

static void __exit mcp9808_exit(void)
{
	i2c_del_driver(&mcp9808_driver);
}

module_init(mcp9808_init);
module_exit(mcp9808_exit);

MODULE_AUTHOR("Alex Deddo");
MODULE_DESCRIPTION("Microchip MCP9808 I2C Temperature Sensor");
MODULE_LICENSE("GPL");
