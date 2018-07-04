#ifndef TEE_MUTEX_WAIT_H
#define TEE_MUTEX_WAIT_H

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/device.h>

struct tee_mutex_wait_private {
	struct mutex mu;
	struct list_head db;
};

int tee_mutex_wait_init(struct tee_mutex_wait_private *priv);
void tee_mutex_wait_exit(struct tee_mutex_wait_private *priv);

void tee_mutex_wait_delete(struct device *dev,
			struct tee_mutex_wait_private *priv,
			u32 key);
void tee_mutex_wait_wakeup(struct device *dev,
			struct tee_mutex_wait_private *priv,
			u32 key, u32 wait_after);
void tee_mutex_wait_sleep(struct device *dev,
			struct tee_mutex_wait_private *priv,
			u32 key, u32 wait_tick);

#endif /*TEE_MUTEX_WAIT_H*/
