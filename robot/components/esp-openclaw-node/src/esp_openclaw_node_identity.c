/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_openclaw_node_identity.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "mbedtls/base64.h"
#include "sha/sha_parallel_engine.h"
#include "nvs.h"
#include "sodium.h"

static const char *TAG = "esp_openclaw_node_identity";
static const char *NVS_NAMESPACE = "openclaw";
static const char *NVS_KEY_SEED = "device_seed";
static const char *NVS_KEY_SESSION_VERSION = "session_v";
static const char *NVS_KEY_SESSION_URI = "session_uri";
static const char *NVS_KEY_SESSION_DEVICE_TOKEN = "session_dev_tok";
#define ESP_OPENCLAW_NODE_DEVICE_AUTH_PAYLOAD_V3_FORMAT \
    "v3|%s|%s|%s|%s|%s|%" PRId64 "|%s|%s|%s|%s"

_Static_assert(ESP_OPENCLAW_NODE_ED25519_SEED_LEN == crypto_sign_SEEDBYTES, "unexpected libsodium seed size");
_Static_assert(
    ESP_OPENCLAW_NODE_ED25519_PUBLIC_KEY_LEN == crypto_sign_PUBLICKEYBYTES,
    "unexpected libsodium public key size");
_Static_assert(
    ESP_OPENCLAW_NODE_ED25519_PRIVATE_KEY_LEN == crypto_sign_SECRETKEYBYTES,
    "unexpected libsodium secret key size");

static esp_err_t ensure_sodium_ready(void)
{
    int rc = sodium_init();
    if (rc < 0) {
        ESP_LOGE(TAG, "libsodium initialization failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void bytes_to_lower_hex(const uint8_t *input, size_t input_len, char *output, size_t output_size)
{
    static const char HEX[] = "0123456789abcdef";
    if (output_size < (input_len * 2U) + 1U) {
        if (output_size > 0) {
            output[0] = '\0';
        }
        return;
    }
    for (size_t i = 0; i < input_len; ++i) {
        output[(i * 2U)] = HEX[(input[i] >> 4) & 0x0f];
        output[(i * 2U) + 1U] = HEX[input[i] & 0x0f];
    }
    output[input_len * 2U] = '\0';
}

static esp_err_t base64url_encode(
    const uint8_t *input,
    size_t input_len,
    char *output,
    size_t output_size)
{
    size_t written = 0;
    int rc = mbedtls_base64_encode((unsigned char *)output, output_size, &written, input, input_len);
    if (rc != 0) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < written; ++i) {
        if (output[i] == '+') {
            output[i] = '-';
        } else if (output[i] == '/') {
            output[i] = '_';
        }
    }
    while (written > 0 && output[written - 1] == '=') {
        --written;
    }
    output[written] = '\0';
    return ESP_OK;
}

static char *normalize_metadata(const char *value)
{
    const char *start = value ? value : "";
    while (*start != '\0' && isspace((unsigned char)*start)) {
        ++start;
    }
    const char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) {
        --end;
    }

    size_t len = (size_t)(end - start);
    char *normalized = calloc(len + 1U, sizeof(char));
    if (normalized == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < len; ++i) {
        normalized[i] = (char)tolower((unsigned char)start[i]);
    }
    normalized[len] = '\0';
    return normalized;
}

static esp_err_t erase_nvs_key_if_present(nvs_handle_t nvs, const char *key)
{
    esp_err_t err = nvs_erase_key(nvs, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    return err;
}

static esp_err_t clear_identity_bound_session_state(nvs_handle_t nvs)
{
    ESP_RETURN_ON_ERROR(erase_nvs_key_if_present(nvs, NVS_KEY_SEED), TAG, "erase malformed seed");
    ESP_RETURN_ON_ERROR(
        erase_nvs_key_if_present(nvs, NVS_KEY_SESSION_VERSION),
        TAG,
        "erase saved session version");
    ESP_RETURN_ON_ERROR(
        erase_nvs_key_if_present(nvs, NVS_KEY_SESSION_URI),
        TAG,
        "erase saved session uri");
    ESP_RETURN_ON_ERROR(
        erase_nvs_key_if_present(nvs, NVS_KEY_SESSION_DEVICE_TOKEN),
        TAG,
        "erase saved session device token");
    return ESP_OK;
}

static esp_err_t generate_and_store_seed(nvs_handle_t nvs, uint8_t *seed, bool *created)
{
    esp_fill_random(seed, ESP_OPENCLAW_NODE_ED25519_SEED_LEN);
    ESP_RETURN_ON_ERROR(
        nvs_set_blob(nvs, NVS_KEY_SEED, seed, ESP_OPENCLAW_NODE_ED25519_SEED_LEN),
        TAG,
        "failed storing device seed");
    *created = true;
    return ESP_OK;
}

static esp_err_t load_seed_from_nvs(nvs_handle_t nvs, uint8_t *seed, bool *created)
{
    size_t required = ESP_OPENCLAW_NODE_ED25519_SEED_LEN;
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_SEED, seed, &required);
    if (err == ESP_OK && required == ESP_OPENCLAW_NODE_ED25519_SEED_LEN) {
        *created = false;
        return ESP_OK;
    }
    if (err == ESP_OK || err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGW(
            TAG,
            "discarding malformed stored identity seed (size=%u) and clearing saved session",
            (unsigned)required);
        ESP_RETURN_ON_ERROR(
            clear_identity_bound_session_state(nvs),
            TAG,
            "clear malformed identity state");
        return generate_and_store_seed(nvs, seed, created);
    }
    if (err != ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    return generate_and_store_seed(nvs, seed, created);
}

esp_err_t esp_openclaw_node_identity_store_seed_if_absent(const uint8_t *seed, size_t seed_len)
{
    if (seed == NULL || seed_len != ESP_OPENCLAW_NODE_ED25519_SEED_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    size_t required = 0;
    err = nvs_get_blob(nvs, NVS_KEY_SEED, NULL, &required);
    if (err == ESP_OK) {
        nvs_close(nvs);
        return ESP_ERR_INVALID_STATE;
    }
    if (err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }

    err = nvs_set_blob(nvs, NVS_KEY_SEED, seed, seed_len);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t esp_openclaw_node_identity_load_or_create(esp_openclaw_node_identity_t *identity)
{
    return esp_openclaw_node_identity_load_or_create_ns(identity, NULL);
}

esp_err_t esp_openclaw_node_identity_load_or_create_ns(
    esp_openclaw_node_identity_t *identity,
    const char *nvs_namespace)
{
    if (identity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A distinct namespace yields a distinct Ed25519 seed -> distinct
     * device_id, letting a second node handle on the SAME chip pair as its own
     * independent device on the gateway. This is what makes a dual-role
     * (operator + node) device possible: the gateway binds one role per device
     * identity, so the two legs need separate identities. NULL keeps the
     * historical default namespace for backward compatibility. */
    const char *ns = (nvs_namespace != NULL && nvs_namespace[0] != '\0')
        ? nvs_namespace
        : NVS_NAMESPACE;

    memset(identity, 0, sizeof(*identity));

    nvs_handle_t nvs = 0;
    esp_err_t err =
        nvs_open(ns, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed opening NVS namespace '%s': %s", ns, esp_err_to_name(err));
        return err;
    }

    bool created = false;
    err = load_seed_from_nvs(nvs, identity->seed, &created);
    if (err != ESP_OK) {
        goto fail;
    }

    err = ensure_sodium_ready();
    if (err != ESP_OK) {
        goto fail;
    }

    if (crypto_sign_seed_keypair(identity->public_key, identity->private_key, identity->seed) != 0) {
        ESP_LOGE(TAG, "failed deriving Ed25519 keypair");
        err = ESP_FAIL;
        goto fail;
    }

    uint8_t digest[32] = {0};
    esp_sha(SHA2_256, identity->public_key, ESP_OPENCLAW_NODE_ED25519_PUBLIC_KEY_LEN, digest);
    bytes_to_lower_hex(digest, sizeof(digest), identity->device_id, sizeof(identity->device_id));
    err = base64url_encode(
        identity->public_key,
        ESP_OPENCLAW_NODE_ED25519_PUBLIC_KEY_LEN,
        identity->public_key_b64url,
        sizeof(identity->public_key_b64url));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed encoding public key: %s", esp_err_to_name(err));
        goto fail;
    }

    err = created ? nvs_commit(nvs) : ESP_OK;
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed committing NVS: %s", esp_err_to_name(err));
        esp_openclaw_node_identity_free(identity);
        memset(identity, 0, sizeof(*identity));
        return err;
    }

    ESP_LOGI(
        TAG,
        "device identity ready: %.12s...",
        identity->device_id);
    return ESP_OK;

fail:
    nvs_close(nvs);
    esp_openclaw_node_identity_free(identity);
    memset(identity, 0, sizeof(*identity));
    return err;
}

void esp_openclaw_node_identity_free(esp_openclaw_node_identity_t *identity)
{
    if (identity == NULL) {
        return;
    }
    memset(identity, 0, sizeof(*identity));
}

esp_err_t esp_openclaw_node_identity_sign_payload(
    const esp_openclaw_node_identity_t *identity,
    const char *payload,
    char *signature_b64url,
    size_t signature_b64url_size)
{
    if (identity == NULL || payload == NULL || signature_b64url == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_sodium_ready(), TAG, "libsodium not ready");

    uint8_t signature[64] = {0};
    unsigned long long signature_len = 0;
    if (crypto_sign_detached(
            signature,
            &signature_len,
            (const unsigned char *)payload,
            strlen(payload),
            identity->private_key) != 0) {
        return ESP_FAIL;
    }
    if (signature_len != sizeof(signature)) {
        return ESP_FAIL;
    }

    return base64url_encode(signature, sizeof(signature), signature_b64url, signature_b64url_size);
}

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
    char **out_payload)
{
    if (identity == NULL || client_id == NULL || client_mode == NULL || role == NULL ||
        scopes_csv == NULL || nonce == NULL || out_payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char *normalized_platform = normalize_metadata(platform);
    char *normalized_family = normalize_metadata(device_family);
    if (normalized_platform == NULL || normalized_family == NULL) {
        free(normalized_platform);
        free(normalized_family);
        return ESP_ERR_NO_MEM;
    }

    const char *safe_token = token ? token : "";
    int required = snprintf(
        NULL,
        0,
        ESP_OPENCLAW_NODE_DEVICE_AUTH_PAYLOAD_V3_FORMAT,
        identity->device_id,
        client_id,
        client_mode,
        role,
        scopes_csv,
        signed_at_ms,
        safe_token,
        nonce,
        normalized_platform,
        normalized_family);
    if (required < 0) {
        free(normalized_platform);
        free(normalized_family);
        return ESP_FAIL;
    }

    char *payload = calloc((size_t)required + 1U, sizeof(char));
    if (payload == NULL) {
        free(normalized_platform);
        free(normalized_family);
        return ESP_ERR_NO_MEM;
    }

    snprintf(
        payload,
        (size_t)required + 1U,
        ESP_OPENCLAW_NODE_DEVICE_AUTH_PAYLOAD_V3_FORMAT,
        identity->device_id,
        client_id,
        client_mode,
        role,
        scopes_csv,
        signed_at_ms,
        safe_token,
        nonce,
        normalized_platform,
        normalized_family);

    free(normalized_platform);
    free(normalized_family);
    *out_payload = payload;
    return ESP_OK;
}
