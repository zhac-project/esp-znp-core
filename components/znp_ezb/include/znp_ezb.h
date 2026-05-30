// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "znp_dispatch.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Returns the singleton backend wired to this chip (IEEE from efuse, reset = esp_restart). */
const znp_backend_t *znp_ezb_backend(void);
#ifdef __cplusplus
}
#endif
