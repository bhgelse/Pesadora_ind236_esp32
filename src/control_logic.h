#pragma once
#include <stdbool.h>

void control_init(void);
void control_update(bool rel_grueso, bool rel_fino, bool rel_descarga);

// Entradas
bool control_boton_inicio(void);
bool control_boton_paro(void);
bool control_pedal_descarga(void);