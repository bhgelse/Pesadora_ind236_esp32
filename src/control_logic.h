#pragma once

#include <stdbool.h>

// Inicializa relés y LEDs
void control_init(void);

// Actualiza salidas. 
// 'permiso' viene del switch Start/Stop.
// Los bool 'manual_*' vienen de los interruptores de accionamiento directo.
void control_update(float peso_actual, float peso_objetivo, bool permiso, 
                    bool manual_grueso, bool manual_fino, bool manual_descarga);