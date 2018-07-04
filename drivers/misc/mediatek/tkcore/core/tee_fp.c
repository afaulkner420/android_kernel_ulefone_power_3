#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "tee_fp.h"

/* initialized to zero by default */
#ifdef CONFIG_MEDIATEK_SOLUTION

#if defined(CONFIG_ARCH_MT6580) || defined(CONFIG_ARCH_MT6570)

void (* enable_spi_clk) (void) = NULL;
void (* disable_spi_clk) (void) = NULL;

#else

struct mt_spi_t *mt_spi = NULL;
void (* enable_spi_clk) (struct mt_spi_t *) = NULL;
void (* disable_spi_clk) (struct mt_spi_t *) = NULL;

#endif

#endif

void tee_fp_enable_spi_clk(void)
{
#ifdef CONFIG_MEDIATEK_SOLUTION
	if (!enable_spi_clk) {
		pr_err("%s() enable_spi_clk() is not configured\n", __func__);
		return;
	}
#if defined(CONFIG_ARCH_MT6580) || defined(CONFIG_ARCH_MT6570)
	enable_spi_clk();
#else
	enable_spi_clk(mt_spi);
#endif
#else
	pr_err("%s() not implemented\n", __func__);
#endif
}
EXPORT_SYMBOL(tee_fp_enable_spi_clk);

void tee_fp_disable_spi_clk(void)
{
#ifdef CONFIG_MEDIATEK_SOLUTION
	if (!disable_spi_clk) {
		pr_err("%s() disable_spi_clk() is not configured\n", __func__);
		return;
	}
#if defined(CONFIG_ARCH_MT6580) || defined(CONFIG_ARCH_MT6570)
	disable_spi_clk();
#else
	disable_spi_clk(mt_spi);
#endif
#else
	pr_err("%s() not implemented\n", __func__);
#endif
}
EXPORT_SYMBOL(tee_fp_disable_spi_clk);

#ifdef CONFIG_MEDIATEK_SOLUTION

#if defined(CONFIG_ARCH_MT6580) || defined(CONFIG_ARCH_MT6570)
int tee_register_spi_clk(void (*enable_fn) (void), void (*disable_fn) (void))
#else
int tee_register_spi_clk(void (*enable_fn) (struct mt_spi_t *),
	void (*disable_fn) (struct mt_spi_t *), struct mt_spi_t *arg)
#endif
{
    if (enable_fn == NULL || disable_fn == NULL) {
        return -EINVAL;
    }

#if defined(CONFIG_ARCH_MT6580) || defined(CONFIG_ARCH_MT6570)
	if (enable_spi_clk || disable_spi_clk) {
		pr_warn("%s() tee_spi_clk re-registered. old enable: %p disable: %p\n",
			__func__, enable_spi_clk, disable_spi_clk);
	}
#else
	if (enable_spi_clk || disable_spi_clk || mt_spi) {
		pr_warn("%s() tee_spi_clk re-registered. old enable: %p disable: %p mt_spi_t: %p\n",
			__func__, enable_spi_clk, disable_spi_clk, mt_spi);
	}
#endif

	enable_spi_clk = enable_fn;
	disable_spi_clk = disable_fn;

#if !defined(CONFIG_ARCH_MT6580) && !defined(CONFIG_ARCH_MT6570)
	mt_spi = arg;
#endif

	return 0;
}

#else

int tee_register_spi_clk(void)
{
	return -1;
}

#endif

EXPORT_SYMBOL(tee_register_spi_clk);

#include <tee_kernel_api.h>

static TEEC_UUID SENSOR_DETECTOR_TA_UUID = { 0x966d3f7c, 0x04ef, 0x1beb, \
    { 0x08, 0xb7, 0x57, 0xf3, 0x7a, 0x6d, 0x87, 0xf9 } };

#define CMD_READ_CHIPID     0x0
#define CMD_DISABLE         0x1

int tee_spi_transfer(void *conf, uint32_t conf_size, void *inbuf, void *outbuf, uint32_t size)
{
	TEEC_Context context;
	TEEC_Session session;
	TEEC_Operation op;

	TEEC_Result r;

	char *buf;


	uint32_t returnOrigin;

	pr_alert("%s conf=%p conf_size=%u inbuf=%p outbuf=%p size=%u\n",
		__func__, conf, conf_size, inbuf, outbuf, size);

	if (!conf || !inbuf || !outbuf) {
		pr_err("Bad parameters NULL buf\n");
		return -EINVAL;
	}

	if (size == 0) {
		pr_err("zero buf size\n");
		return -EINVAL;
	}

	memset(&context, 0, sizeof(context));
	memset(&session, 0, sizeof(session));
	memset(&op, 0, sizeof(op));

	memcpy(outbuf, inbuf, size);

	if ((r = TEEC_InitializeContext(NULL, &context)) != TEEC_SUCCESS) {
		pr_err("TEEC_InitializeContext() failed with 0x%08x\n", r);
		return r;
	}
	if ((r = TEEC_OpenSession(
		&context, &session, &SENSOR_DETECTOR_TA_UUID,
		TEEC_LOGIN_PUBLIC,
		NULL, NULL, &returnOrigin)) != TEEC_SUCCESS) {
		pr_err("%s TEEC_OpenSession failed with 0x%x returnOrigun: %u\n",
			__func__, r, returnOrigin);
		TEEC_FinalizeContext(&context);
		return r;
	}

	op.paramTypes = TEEC_PARAM_TYPES(
		TEEC_MEMREF_TEMP_INPUT,
		TEEC_MEMREF_TEMP_INOUT,
		TEEC_NONE,
		TEEC_NONE);

	op.params[0].tmpref.buffer = conf;
	op.params[0].tmpref.size = conf_size;

	op.params[1].tmpref.buffer = outbuf;
	op.params[1].tmpref.size = size;

	buf = outbuf;

	if ((r = TEEC_InvokeCommand(&session, CMD_READ_CHIPID, &op, &returnOrigin)) != TEEC_SUCCESS) {
		pr_err("%s TEEC_InvokeCommand() failed with 0x%08x returnOrigin: %u\n",
			__func__, r, returnOrigin);
	}

	TEEC_CloseSession(&session);
	TEEC_FinalizeContext(&context);

	pr_alert("[0x%02x 0x%02x 0x%02x 0x%02x]\n",
		buf[0], buf[1], buf[2], buf[3]);

	return r;
}

EXPORT_SYMBOL(tee_spi_transfer);

int tee_spi_transfer_disable(void)
{
	TEEC_Context context;
	TEEC_Session session;
	TEEC_Operation op;

	TEEC_Result r;

	uint32_t returnOrigin;

	memset(&context, 0, sizeof(context));
	memset(&session, 0, sizeof(session));
	memset(&op, 0, sizeof(op));

	if ((r = TEEC_InitializeContext(NULL, &context)) != TEEC_SUCCESS) {
		pr_err("TEEC_InitializeContext() failed with 0x%08x\n", r);
		return r;
	}

	if ((r = TEEC_OpenSession(
		&context, &session, &SENSOR_DETECTOR_TA_UUID,
		TEEC_LOGIN_PUBLIC,
		NULL, NULL, &returnOrigin)) != TEEC_SUCCESS) {
		pr_err("%s TEEC_OpenSession failed with 0x%x returnOrigun: %u\n",
			__func__, r, returnOrigin);
		TEEC_FinalizeContext(&context);
		return r;
	}

	op.paramTypes = TEEC_PARAM_TYPES(
		TEEC_NONE,
		TEEC_NONE,
		TEEC_NONE,
		TEEC_NONE);

	if ((r = TEEC_InvokeCommand(&session, CMD_DISABLE, &op, &returnOrigin)) != TEEC_SUCCESS) {
		pr_err("%s TEEC_InvokeCommand() failed with 0x%08x returnOrigin: %u\n",
			__func__, r, returnOrigin);
	}

	TEEC_CloseSession(&session);
	TEEC_FinalizeContext(&context);

	return r;
}

EXPORT_SYMBOL(tee_spi_transfer_disable);

int tee_fp_init(void)
{
	return 0;
}

void tee_fp_exit(void)
{
}
