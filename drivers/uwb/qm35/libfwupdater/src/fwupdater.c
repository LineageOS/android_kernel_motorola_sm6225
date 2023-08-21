
#ifndef __KERNEL__
#include <stddef.h>
#endif

#include <linux/uwb/qmrom_spi.h>
#include <linux/uwb/qmrom_log.h>
#include <linux/uwb/qmrom_utils.h>
#include <linux/uwb/spi_rom_protocol.h>

#include <linux/uwb/fwupdater.h>

/* Extract from C0 rom code */
#define MAX_CERTIFICATE_SIZE 0x400
#define MAX_CHUNK_SIZE 4096
#define WAIT_SS_RDY_TIMEOUT 100
#define RESULT_RETRIES 3
#define RESULT_CMD_INTERVAL_MS 50

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#ifndef __KERNEL__
_Static_assert(sizeof(struct stc) + MAX_CERTIFICATE_SIZE < MAX_CHUNK_SIZE);
#endif

static int send_data_chunks(struct qmrom_handle *handle, char *data,
			    size_t size);

int run_fwupdater(struct qmrom_handle *handle, char *fwpkg_bin, size_t size)
{
	int rc;

	if (size < sizeof(struct fw_pkg_hdr_t) +
			   sizeof(struct fw_pkg_img_hdr_t) +
			   CRYPTO_IMAGES_CERT_PKG_SIZE +
			   CRYPTO_FIRMWARE_CHUNK_MIN_SIZE) {
		LOG_ERR("%s cannot extract enough data from fw package binary\n",
			__func__);
		return -EINVAL;
	}

	rc = send_data_chunks(handle, fwpkg_bin, size);
	if (rc) {
		LOG_ERR("Sending image failed with %d\n", rc);
		return rc;
	}
	return 0;
}

static int run_fwupdater_get_status(struct qmrom_handle *handle,
				    struct stc *hstc, struct stc *sstc,
				    struct fw_updater_status_t *status)
{
	uint32_t i = 0;
	bool owa;

	while (i++ < RESULT_RETRIES) {
		// Poll the QM
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc, sizeof(hstc));
		// LOG_INFO("Poll received:\n");
		// hexdump(LOG_INFO, sstc, sizeof(sstc));
		/* Line might have been read ready before QM actually handled the previous cmd */
		qmrom_msleep(RESULT_CMD_INTERVAL_MS);

		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.pre_read = 1;
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc, sizeof(hstc));
		// LOG_INFO("Pre-Read received:\n");
		// hexdump(LOG_INFO, sstc, sizeof(sstc));

		/* Stops the loop when QM has a result to share */
		owa = sstc->soc_flags.out_waiting;
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_TIMEOUT);
		sstc->all = 0;
		hstc->all = 0;
		hstc->host_flags.read = 1;
		hstc->len = sizeof(*status);
		qmrom_spi_transfer(handle->spi_handle, (char *)sstc,
				   (const char *)hstc,
				   sizeof(hstc) + sizeof(*status));
		// LOG_INFO("Read received:\n");
		// hexdump(LOG_INFO, sstc, sizeof(hstc) + sizeof(uint32_t));
		if (owa) {
			memcpy(status, sstc->payload, sizeof(*status));
			break;
		}
	}
	if (!owa) {
		LOG_ERR("Timedout waiting for result\n");
		return -1;
	}
	return 0;
}

static struct stc *prepare_hstc(bool copy_data, char *tx, char *data, size_t len)
{
	struct stc *hstc = (struct stc *)(copy_data ? tx : data);
	hstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->len = len;
	if (copy_data)
		memcpy(hstc->payload, data, len);
	return hstc;
}

static int send_data_chunks(struct qmrom_handle *handle, char *data,
			    size_t size)
{
	struct fw_updater_status_t status;
	char *rx, *tx;
	struct stc *hstc, *sstc;
	size_t to_send;
	int rc = 0;
	bool copy_data = !qmrom_data_dma_able(data);

	qmrom_alloc(rx, MAX_CHUNK_SIZE + sizeof(struct stc));
	qmrom_alloc(tx, MAX_CHUNK_SIZE + sizeof(struct stc));
	if (!rx || !tx) {
		LOG_ERR("Rx/Tx buffers allocation failure\n");
		if (rx)
			qmrom_free(rx);
		if (tx)
			qmrom_free(tx);
		return -1;
	}

	hstc = (struct stc *)tx;
	sstc = (struct stc *)rx;

	/* Sending the fw package header */
	LOG_INFO("Sending the fw package header (copy_data is %d)\n", copy_data);
	hstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->len = sizeof(struct fw_pkg_hdr_t);
	size -= sizeof(struct fw_pkg_hdr_t);
	memcpy(&tx[sizeof(struct stc)], data, sizeof(struct fw_pkg_hdr_t));
	/* Move the data to the next offset minus the stc footprint */
	data += sizeof(struct fw_pkg_hdr_t) - (copy_data ? 0 : sizeof(struct stc));
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	rc = qmrom_spi_transfer(handle->spi_handle, rx, tx,
			   sizeof(struct fw_pkg_hdr_t) + sizeof(struct stc));
	if (rc)
	{
		LOG_ERR("First qmrom_spi_transfer failed with %d\n", rc);
		return rc;
	}
	// hexdump(LOG_INFO, tx + sizeof(struct stc), sizeof(struct fw_pkg_hdr_t));

	/* Sending the image header */
	LOG_INFO("Sending the image header\n");
	hstc = prepare_hstc(copy_data, tx, data, sizeof(struct fw_pkg_img_hdr_t));
	size -= sizeof(struct fw_pkg_img_hdr_t);
	// hexdump(LOG_INFO, data + sizeof(struct stc),
	// 	sizeof(struct fw_pkg_img_hdr_t));
	data += sizeof(struct fw_pkg_img_hdr_t);
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	qmrom_spi_transfer(handle->spi_handle, rx, (const char *)hstc,
			   sizeof(struct fw_pkg_img_hdr_t) +
				   sizeof(struct stc));

	/* Sending the cert chain */
	LOG_INFO("Sending the cert chain\n");
#if MAX_CHUNK_SIZE < CRYPTO_IMAGES_CERT_PKG_SIZE
	hstc = prepare_hstc(true, tx, data, CRYPTO_IMAGES_CERT_KEY_SIZE);
	size -= CRYPTO_IMAGES_CERT_KEY_SIZE;
	data += CRYPTO_IMAGES_CERT_KEY_SIZE;
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	qmrom_spi_transfer(handle->spi_handle, rx, tx,
			   CRYPTO_IMAGES_CERT_KEY_SIZE);

	hstc = prepare_hstc(true, tx, data, CRYPTO_IMAGES_CERT_KEY_SIZE);
	size -= CRYPTO_IMAGES_CERT_KEY_SIZE;
	data += CRYPTO_IMAGES_CERT_KEY_SIZE;
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	qmrom_spi_transfer(handle->spi_handle, rx, tx,
			   CRYPTO_IMAGES_CERT_KEY_SIZE);

	hstc = prepare_hstc(true, tx, data, CRYPTO_IMAGES_CERT_CONTENT_SIZE);
	size -= CRYPTO_IMAGES_CERT_CONTENT_SIZE;
	data += CRYPTO_IMAGES_CERT_CONTENT_SIZE;
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	qmrom_spi_transfer(handle->spi_handle, rx, tx,
			   CRYPTO_IMAGES_CERT_CONTENT_SIZE);
#else
	hstc = prepare_hstc(copy_data, tx, data, CRYPTO_IMAGES_CERT_PKG_SIZE);
	size -= CRYPTO_IMAGES_CERT_PKG_SIZE;
	// hexdump(LOG_INFO, data + sizeof(struct stc), 100);
	data += CRYPTO_IMAGES_CERT_PKG_SIZE;
	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);
	qmrom_spi_transfer(handle->spi_handle, rx, (const char *)hstc,
			   CRYPTO_IMAGES_CERT_PKG_SIZE + sizeof(struct stc));
#endif

	/* Sending the fw image */
	LOG_INFO("Sending the image\n");
	do {
		to_send = MIN(MAX_CHUNK_SIZE, size);
		LOG_DBG("Sending a chunk of %zu bytes\n", to_send);
		hstc = prepare_hstc(copy_data, tx, data, to_send);
		qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
					      WAIT_SS_RDY_TIMEOUT);
		qmrom_spi_transfer(handle->spi_handle, rx, (const char *)hstc,
				   to_send + sizeof(struct stc));
		data += to_send;
		size -= to_send;
	} while (size);

	qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle,
				      WAIT_SS_RDY_TIMEOUT);

	rc = run_fwupdater_get_status(handle, hstc, sstc, &status);
	if (rc) {
		LOG_ERR("run_fwupdater_get_status returned %d\n", rc);
	} else {
		if (status.status) {
			LOG_ERR("%s failed, fw updater status %#x (suberror %#x)\n",
				__func__, status.status, status.suberror);
			rc = status.status;
		} else {
			LOG_INFO("%s succeeded\n", __func__);
		}
	}

	if (rx)
		qmrom_free(rx);
	if (tx)
		qmrom_free(tx);
	return rc;
}
