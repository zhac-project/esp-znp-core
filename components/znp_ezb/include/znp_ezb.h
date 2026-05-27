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
