/*
* Copyright 2021 Qorvo US, Inc.
*
* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
*
* This file is provided under the Apache License 2.0, or the
* GNU General Public License v2.0.
*
*/

#ifndef _QM358XX_FIRMWARE_PACK_HANDLING_H_
#define _QM358XX_FIRMWARE_PACK_HANDLING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*!
@addtogroup secureboot_firmware_pack_handling
@{
*/

/*! Cert magic word conversion macro */
#define MAGIC_STR_TO_U32(x) ((x[3]) | (x[2] << 8) | (x[1] << 16) | (x[0] << 24))

/*! Cert version word conversion macro */
#define CERT_VER_TO_U32(maj, min) (((maj) << 16) | ((min) << 24))

// Field values for firmware package
#define CRYPTO_FIRMWARE_PACK_MAGIC_VALUE MAGIC_STR_TO_U32("CFWP")

// Package size defines
#define CRYPTO_FIRMWARE_PACK_ENC_KEY_L2_ENC_SIZE 64u
#define CRYPTO_FIRMWARE_PACK_ENC_DATA_SIZE 16u
#define CRYPTO_FIRMWARE_PACK_FW_VERSION_SIZE 32u
#define CRYPTO_FIRMWARE_PACK_TAG_SIZE 16u

// Firmware image size define
#define CRYPTO_FW_PKG_IMG_HDR_IMG_NUM_MAX 8u

#define CRYPTO_FIRMWARE_PACK_HEADER_SIZE sizeof(fw_pkg_hdr_t)
#define CRYPTO_FIRMWARE_IMAGE_CERT_SIZE sizeof(fw_pkg_img_hdr_t)

// Field values for firmware chunks
#define CRYPTO_FIRMWARE_CHUNK_MAGIC_VALUE MAGIC_STR_TO_U32("CFWC")
#define CRYPTO_FIRMWARE_CHUNK_VERSION CERT_VER_TO_U32(1, 0)
#define CRYPTO_FIRMWARE_CHUNK_MIN_SIZE 16u
#define CRYPTO_FIRMWARE_CHUNK_HDR_SIZE sizeof(fw_pkg_payload_chunk_hdr_t)

/*! Firmware Package Header fields */
typedef struct {
	uint32_t magic; /**< Magic number. */
	uint32_t version; /**< Version. */
	uint8_t package_type; /**< Package type. */
	uint8_t enc_mode; /**< Encryption mode. */
	uint8_t enc_algo; /**< Encryption algorithm. */
	union {
		uint8_t enc_key_l2_cfg; /**< Code encryption L2 key config. */
		struct {
			uint8_t use : 1; /**< Use Kce_l2 for decryption. */
			uint8_t lock : 1; /**< Lock using Kce_l2 for decryption. */
			uint8_t : 6;
		} enc_key_l2_cfg_b;
	};
	uint8_t enc_data
		[CRYPTO_FIRMWARE_PACK_ENC_DATA_SIZE]; /**< Encryption data. */
	uint8_t enc_key_l2
		[CRYPTO_FIRMWARE_PACK_ENC_KEY_L2_ENC_SIZE]; /**< Code encryption L2 key data. */
	uint8_t fw_version
		[CRYPTO_FIRMWARE_PACK_FW_VERSION_SIZE]; /**< Firwmare version included in the package. */
	uint32_t payload_len; /**< Payload length. */
	uint8_t tag[CRYPTO_FIRMWARE_PACK_TAG_SIZE]; /**< AES-CMAC Tag. */
} fw_pkg_hdr_t;

/*! Firmware Image Metadata fields */
typedef struct {
	uint32_t offset; /**< Offset. */
	uint32_t length; /**< Length. */
} fw_image_metadata_t;

/*! Firmware Image Header fields */
typedef struct {
	uint32_t magic; /**< Magic number. */
	uint32_t version; /**< Version number. */
	uint32_t cert_chain_offset; /**< Certificate chain offset. */
	uint16_t cert_chain_length; /**< Certificate chain length. */
	uint8_t img_num; /**< Number of images. */
	uint8_t reserved; /**< Reserved data. */
	fw_image_metadata_t metadata
		[CRYPTO_FW_PKG_IMG_HDR_IMG_NUM_MAX]; /**< Firmware image metadata. */
} fw_pkg_img_hdr_t;

/*! Firmware Chunk fields */
typedef struct {
	uint32_t magic; /**< Magic number. */
	uint32_t version; /**< Version number. */
	uint32_t length; /**< Length. */
} fw_pkg_payload_chunk_hdr_t;

#define CRYPTO_IMAGES_CERT_PUBLIC_KEY_SIZE 384
#define CRYPTO_KEY_HASH_TRUNC_SIZE_BYTES 16u
#define CRYPTO_SHA256_SIZE_BYTES 32u
#define CRYPTO_IMAGES_CERT_MIN_IMAGE_NUMBER 1u
#define CRYPTO_IMAGES_CERT_PKG_SIZE sizeof(cert_chain_struct)

/*! Certificate or file header struct */
typedef struct {
	uint32_t magic; /**< Magic value. */
	uint32_t version; /**< Certificate version. */
	uint32_t size; /**< Certificate size. */
} cert_common_header_struct;

/*! Key certificate fields */
typedef struct {
	cert_common_header_struct
		header; /**< Pointer to key certificate header data. */
	uint8_t rsa_public_key
		[CRYPTO_IMAGES_CERT_PUBLIC_KEY_SIZE]; /**< RSA public key data. */
	uint32_t context_hash_id; /**< Context and hash ID combined in one uint32. */
	uint32_t customer_id; /**< Customer ID. */
	uint32_t product_id; /**< Product ID. */
	uint8_t sec_version; /**< Security version number. */
	uint8_t reserved[3]; /**< Reserved data. */
	uint32_t arb_version; /**< Anti-rollback version number. */
	uint8_t hash[CRYPTO_KEY_HASH_TRUNC_SIZE_BYTES]; /**< Content cert public key hash. */
	uint8_t rsa_signature
		[CRYPTO_IMAGES_CERT_PUBLIC_KEY_SIZE]; /**< RSA signature data. */
} cert_key_struct;

/*! Content of metadata fields */
typedef struct {
	uint8_t img_hash[CRYPTO_SHA256_SIZE_BYTES]; /**< Image hash result data. */
	uint32_t flash_addr; /**< Flash (RRAM) address. */
	uint32_t max_len; /**< Maximum length. */
	uint32_t len; /**< Length. */
	uint32_t load_verify_scheme; /**< Schema to load and verify firmware image. */
	uint32_t load_addr; /**< SRAM load address. */
} cert_content_img_metadata_t;

/*! Content certificate fields */
typedef struct {
	cert_common_header_struct
		header; /**< Pointer to content certificate header data. */
	uint8_t rsa_public_key
		[CRYPTO_IMAGES_CERT_PUBLIC_KEY_SIZE]; /**< RSA public key data. */
	uint32_t context; /**< Context. */
	uint32_t customer_id; /**< Customer ID. */
	uint32_t product_id; /**< Product ID. */
	uint8_t sec_version; /**< Security version number. */
	uint8_t reserved[3]; /**< Reserved data. */
	uint32_t arb_version; /**< Anti-rollback version number. */
	uint32_t img_num; /**< Number of images. */
	cert_content_img_metadata_t img_metadata
		[CRYPTO_IMAGES_CERT_MIN_IMAGE_NUMBER]; /**< Image metadata. */
} cert_content_struct;

/*! Certificate chain fields */
typedef struct {
	cert_key_struct cert_key; /**< Pointer to key certificate data. */
	cert_content_struct
		cert_content; /**< Pointer to content certificate data. */
} cert_chain_struct;

/**
@}
 */

#ifdef __cplusplus
}
#endif

#endif /* _QM358XX_FIRMWARE_PACK_HANDLING_H_ */
