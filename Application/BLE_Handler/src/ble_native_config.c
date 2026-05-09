/**
 * @file ble_native_config.c
 * @brief JSON config parser for BLE Native Mesh handler.
 *
 * Parses the server-sent "CFBN:JSON:<stack_id>:<json>" payload into the
 * internal ble_native_stack_config_t structures.  All network keys, command
 * names and opcodes come from JSON — nothing is hardcoded here.
 */

#include "ble_native_config.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ble_native_cfg";

/* Module-private config store — one entry per supported stack */
static ble_native_stack_config_t s_cfg[BLE_NATIVE_MAX_STACKS];

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief Convert a hex string like "A1B2C3D4..." into a byte array.
 *
 * @param hex_str  Null-terminated hex string (32 chars for 16-byte key)
 * @param out      Output buffer
 * @param out_len  Expected byte count (e.g. 16)
 * @return true on success
 */
static bool parse_hex_key(const char *hex_str, uint8_t *out, size_t out_len) {
    if (!hex_str || strlen(hex_str) < out_len * 2) {
        return false;
    }
    for (size_t i = 0; i < out_len; i++) {
        char byte_str[3] = { hex_str[i * 2], hex_str[i * 2 + 1], '\0' };
        char *endptr;
        unsigned long val = strtoul(byte_str, &endptr, 16);
        if (endptr != byte_str + 2) {
            return false;
        }
        out[i] = (uint8_t)val;
    }
    return true;
}

/**
 * @brief Parse a uint string that may start with "0x" or plain decimal.
 */
static uint32_t parse_uint(const char *s) {
    if (!s) return 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        return (uint32_t)strtoul(s + 2, NULL, 16);
    }
    return (uint32_t)strtoul(s, NULL, 10);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_native_config_load(uint8_t stack_id,
                                  const char *json_str,
                                  uint16_t json_len) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS || !json_str || json_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_ParseWithLength(json_str, json_len);
    if (!root) {
        ESP_LOGE(TAG, "stack=%u: JSON parse failed near: %.20s",
                 stack_id, cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "?");
        return ESP_FAIL;
    }

    ble_native_stack_config_t *cfg = &s_cfg[stack_id];
    memset(cfg, 0, sizeof(*cfg));
    cfg->stack_id = stack_id;

    /* ---- ble_native object ---- */
    cJSON *ble_obj = cJSON_GetObjectItemCaseSensitive(root, "ble_native");
    if (!ble_obj) {
        ESP_LOGE(TAG, "stack=%u: missing \"ble_native\" key", stack_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    /* ---- Mesh sub-object ---- */
    cJSON *mesh_obj = cJSON_GetObjectItemCaseSensitive(ble_obj, "mesh");
    if (!mesh_obj) {
        ESP_LOGE(TAG, "stack=%u: missing \"mesh\" in ble_native", stack_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    ble_native_mesh_cfg_t *mc = &cfg->mesh;

    cJSON *j_name = cJSON_GetObjectItemCaseSensitive(mesh_obj, "provisioner_name");
    if (cJSON_IsString(j_name)) {
        strncpy(mc->provisioner_name, j_name->valuestring,
                sizeof(mc->provisioner_name) - 1);
    }

    cJSON *j_nk = cJSON_GetObjectItemCaseSensitive(mesh_obj, "net_key");
    if (!cJSON_IsString(j_nk) || !parse_hex_key(j_nk->valuestring, mc->net_key, 16)) {
        ESP_LOGE(TAG, "stack=%u: invalid or missing net_key", stack_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *j_ak = cJSON_GetObjectItemCaseSensitive(mesh_obj, "app_key");
    if (!cJSON_IsString(j_ak) || !parse_hex_key(j_ak->valuestring, mc->app_key, 16)) {
        ESP_LOGE(TAG, "stack=%u: invalid or missing app_key", stack_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *j_ttl = cJSON_GetObjectItemCaseSensitive(mesh_obj, "ttl");
    mc->ttl = cJSON_IsNumber(j_ttl) ? (uint8_t)j_ttl->valuedouble : 7;

    cJSON *j_ua = cJSON_GetObjectItemCaseSensitive(mesh_obj, "primary_unicast_addr");
    mc->primary_unicast_addr = cJSON_IsNumber(j_ua) ? (uint16_t)j_ua->valuedouble : 1;
    /* First node assigned address starts from provisioner_addr + 1 */
    mc->next_unicast_addr = mc->primary_unicast_addr + 1;
    mc->valid = true;

    /* ---- Commands array ---- */
    cJSON *cmds = cJSON_GetObjectItemCaseSensitive(ble_obj, "commands");
    if (cJSON_IsArray(cmds)) {
        int count = cJSON_GetArraySize(cmds);
        if (count > BLE_NATIVE_MAX_COMMANDS) {
            ESP_LOGW(TAG, "stack=%u: commands truncated to %d", stack_id, BLE_NATIVE_MAX_COMMANDS);
            count = BLE_NATIVE_MAX_COMMANDS;
        }
        for (int i = 0; i < count; i++) {
            cJSON *entry = cJSON_GetArrayItem(cmds, i);
            if (!entry) continue;
            ble_native_cmd_entry_t *ce = &cfg->commands[i];

            cJSON *j_cname = cJSON_GetObjectItemCaseSensitive(entry, "name");
            if (cJSON_IsString(j_cname)) {
                strncpy(ce->name, j_cname->valuestring, sizeof(ce->name) - 1);
            }

            cJSON *j_mid = cJSON_GetObjectItemCaseSensitive(entry, "model_id");
            if (cJSON_IsString(j_mid)) {
                ce->model_id = (uint16_t)parse_uint(j_mid->valuestring);
            }

            cJSON *j_op = cJSON_GetObjectItemCaseSensitive(entry, "opcode");
            if (cJSON_IsString(j_op)) {
                ce->opcode = parse_uint(j_op->valuestring);
            }

            cJSON *j_amid = cJSON_GetObjectItemCaseSensitive(entry, "ack_model_id");
            if (cJSON_IsString(j_amid)) {
                ce->ack_model_id = (uint16_t)parse_uint(j_amid->valuestring);
            }

            cJSON *j_aop = cJSON_GetObjectItemCaseSensitive(entry, "ack_opcode");
            if (cJSON_IsString(j_aop)) {
                ce->ack_opcode = parse_uint(j_aop->valuestring);
            }

            cJSON *j_schema = cJSON_GetObjectItemCaseSensitive(entry, "param_schema");
            if (cJSON_IsString(j_schema)) {
                strncpy(ce->param_schema, j_schema->valuestring,
                        sizeof(ce->param_schema) - 1);
            }

            ce->valid = (ce->name[0] != '\0');
            if (ce->valid) {
                ESP_LOGD(TAG, "  cmd[%d]: name=%s model=0x%04X op=0x%06X",
                         i, ce->name, ce->model_id, ce->opcode);
            }
        }
        cfg->num_commands = (uint8_t)count;
    }

    cfg->loaded = true;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "stack=%u: loaded (name='%s', cmds=%u)",
             stack_id, mc->provisioner_name, cfg->num_commands);
    return ESP_OK;
}

esp_err_t ble_native_config_find_cmd(uint8_t stack_id,
                                      const char *name,
                                      ble_native_cmd_entry_t *out) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS || !name || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    ble_native_stack_config_t *cfg = &s_cfg[stack_id];
    for (uint8_t i = 0; i < cfg->num_commands; i++) {
        if (cfg->commands[i].valid &&
            strncmp(cfg->commands[i].name, name, BLE_NATIVE_CMD_NAME_LEN) == 0) {
            memcpy(out, &cfg->commands[i], sizeof(*out));
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ble_native_config_get_mesh(uint8_t stack_id, ble_native_mesh_cfg_t *out) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_cfg[stack_id].mesh.valid) {
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(out, &s_cfg[stack_id].mesh, sizeof(*out));
    return ESP_OK;
}

bool ble_native_config_is_loaded(uint8_t stack_id) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS) return false;
    return s_cfg[stack_id].loaded;
}

esp_err_t ble_native_config_alloc_unicast(uint8_t stack_id, uint16_t *addr_out) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS || !addr_out) {
        return ESP_ERR_INVALID_ARG;
    }
    ble_native_mesh_cfg_t *mc = &s_cfg[stack_id].mesh;
    if (!mc->valid) {
        return ESP_ERR_INVALID_STATE;
    }
    *addr_out = mc->next_unicast_addr;
    mc->next_unicast_addr++;
    return ESP_OK;
}

uint8_t ble_native_config_get_num_cmds(uint8_t stack_id) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS) return 0;
    return s_cfg[stack_id].num_commands;
}

esp_err_t ble_native_config_get_cmd_by_index(uint8_t stack_id,
                                               uint8_t index,
                                               ble_native_cmd_entry_t *out) {
    if (stack_id >= BLE_NATIVE_MAX_STACKS || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (index >= s_cfg[stack_id].num_commands) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(out, &s_cfg[stack_id].commands[index], sizeof(*out));
    return ESP_OK;
}
