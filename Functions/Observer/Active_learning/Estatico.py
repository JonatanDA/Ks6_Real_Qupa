def process(obs_ang_offset_global: float, updated_anterior_indices: dict, app_x_m: float = 0.0, app_y_m: float = 0.0):
    # =========================================================================
    # MODO ESTÁTICO (OBSERVACIÓN PURA)
    # El robot no realiza ninguna acción de movimiento ni evade obstáculos.
    # Toda la capacidad de procesamiento se dedica a trackear a los maestros.
    # =========================================================================
    return None


def reset_history():
    pass


class ActiveLearning:
    process = staticmethod(process)
    reset_history = staticmethod(reset_history)
