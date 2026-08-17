/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define ESP_OPENCLAW_NODE_ED25519_SEED_LEN              32
#define ESP_OPENCLAW_NODE_ED25519_PUBLIC_KEY_LEN        32
#define ESP_OPENCLAW_NODE_ED25519_PRIVATE_KEY_LEN       64
#define ESP_OPENCLAW_NODE_DEVICE_ID_HEX_LEN             64
#define ESP_OPENCLAW_NODE_PUBLIC_KEY_B64URL_LEN         43
#define ESP_OPENCLAW_NODE_SIGNATURE_B64URL_LEN          86
#define ESP_OPENCLAW_NODE_PUBLIC_KEY_B64_ENCODED_LEN    44
#define ESP_OPENCLAW_NODE_SIGNATURE_B64_ENCODED_LEN     88
#define ESP_OPENCLAW_NODE_PUBLIC_KEY_B64_BUFFER_LEN     (ESP_OPENCLAW_NODE_PUBLIC_KEY_B64_ENCODED_LEN + 1)
#define ESP_OPENCLAW_NODE_SIGNATURE_B64_BUFFER_LEN      (ESP_OPENCLAW_NODE_SIGNATURE_B64_ENCODED_LEN + 1)

/** @brief Persisted and derived device identity state. */
typedef struct {
    uint8_t seed[ESP_OPENCLAW_NODE_ED25519_SEED_LEN];                  /**< Ed25519 seed stored in NVS. */
    uint8_t public_key[ESP_OPENCLAW_NODE_ED25519_PUBLIC_KEY_LEN];     /**< Derived public key bytes. */
    uint8_t private_key[ESP_OPENCLAW_NODE_ED25519_PRIVATE_KEY_LEN];   /**< Derived private key bytes. */
    char device_id[ESP_OPENCLAW_NODE_DEVICE_ID_HEX_LEN + 1];          /**< Stable hex device identifier. */
    char public_key_b64url[ESP_OPENCLAW_NODE_PUBLIC_KEY_B64_BUFFER_LEN]; /**< Base64url-encoded public key. */
} esp_openclaw_node_identity_t;

/**
 * @brief Load the node identity from NVS or create and persist a new one.
 *
 * @param[out] identity Identity struct to populate.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an error code if loading or generation fails
 */
esp_err_t esp_openclaw_node_identity_load_or_create(esp_openclaw_node_identity_t *identity);

/**
 * @brief Load or create a node identity from a specific NVS namespace.
 *
 * A distinct namespace produces a distinct Ed25519 seed and therefore a
 * distinct `device_id`, which lets two node handles on the same chip present
 * as independent devices to the gateway (required for a simultaneous
 * operator + node dual-role device, since the gateway binds one role per
 * device identity).
 *
 * @param[out] identity Identity struct to populate.
 * @param[in] nvs_namespace NVS namespace to use, or NULL/empty for the default.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an error code if loading or generation fails
 */
esp_err_t esp_openclaw_node_identity_load_or_create_ns(
    esp_openclaw_node_identity_t *identity,
    const char *nvs_namespace);

/**
 * @brief Persist a caller-provided Ed25519 seed when no identity exists yet.
 *
 * This helper provisions the seed used by @ref esp_openclaw_node_identity_load_or_create
 * before the first node identity is created. It never overwrites an existing
 * stored seed.
 *
 * @param[in] seed Seed bytes to persist.
 * @param[in] seed_len Length of @p seed in bytes. Must equal
 *        @ref ESP_OPENCLAW_NODE_ED25519_SEED_LEN.
 *
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if the seed is missing or the wrong length
 *      - `ESP_ERR_INVALID_STATE` if a seed is already provisioned
 *      - another error code if persistence fails
 */
esp_err_t esp_openclaw_node_identity_store_seed_if_absent(const uint8_t *seed, size_t seed_len);

/**
 * @brief Release dynamically allocated identity fields.
 *
 * @param[in] identity Identity struct to clean up.
 */
void esp_openclaw_node_identity_free(esp_openclaw_node_identity_t *identity);

/**
 * @brief Sign the canonical device-auth payload and return it as base64url text.
 *
 * @param[in] identity Identity state with the private key.
 * @param[in] payload Canonical auth payload to sign.
 * @param[out] signature_b64url Output buffer for the base64url signature.
 * @param[in] signature_b64url_size Size of @p signature_b64url in bytes.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an error code if signing or encoding fails
 */
esp_err_t esp_openclaw_node_identity_sign_payload(
    const esp_openclaw_node_identity_t *identity,
    const char *payload,
    char *signature_b64url,
    size_t signature_b64url_size);

/**
 * @brief Build the protocol v3 device-auth payload string prior to signing.
 *
 * @param[in] identity Identity state.
 * @param[in] client_id Client identifier.
 * @param[in] client_mode Client mode string.
 * @param[in] role Gateway role string.
 * @param[in] scopes_csv Comma-separated scopes string.
 * @param[in] signed_at_ms Millisecond timestamp for the signature.
 * @param[in] token Token value included in the signed payload when applicable.
 * @param[in] nonce Gateway challenge nonce.
 * @param[in] platform Platform metadata.
 * @param[in] device_family Device-family metadata.
 * @param[out] out_payload Allocated payload string to sign.
 *
 * @return
 *      - `ESP_OK` on success
 *      - an error code if payload construction fails
 */
esp_err_t esp_openclaw_node_identity_build_auth_payload_v3(
    const esp_openclaw_node_identity_t *identity,
    const char *client_id,
    const char *client_mode,
    const char *role,
    const char *scopes_csv,
    int64_t signed_at_ms,
    const char *token,
    const char *nonce,
    const char *platform,
    const char *device_family,
    char **out_payload);
