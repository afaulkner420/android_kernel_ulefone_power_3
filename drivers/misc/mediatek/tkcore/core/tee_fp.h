#ifndef TEE_FP_H
#define TEE_FP_H

void tee_fp_enable_spi_clk(void);

void tee_fp_disable_spi_clk(void);

int tee_fp_init(void);

void tee_fp_exit(void);

int tee_spi_transfer(void *conf, uint32_t conf_size, void *inbuf, void *outbuf, uint32_t size);
int tee_spi_transfer_disable(void);

#endif
