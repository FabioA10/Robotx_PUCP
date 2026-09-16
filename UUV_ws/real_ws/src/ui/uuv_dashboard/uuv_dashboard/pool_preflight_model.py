"""Pure Go/No-Go checks for a read-only UUV pool session."""


REQUIRED_CHECKS = (
    'vehicle_inspected',
    'tether_clear',
    'area_clear',
    'recovery_ready',
    'recording_ready',
)


def evaluate(manual_checks, *, connected, armed, output_enabled,
             camera_required=False, camera_available=False):
    """Return ``(ready, reasons)`` without changing vehicle state.

    A ready result only permits the observation/recording session described by
    the read-only launch.  It is never authorization to arm or move the ROV.
    """
    missing = [name for name in REQUIRED_CHECKS if not manual_checks.get(name)]
    reasons = []
    if missing:
        reasons.append('Faltan comprobaciones manuales: ' + ', '.join(missing))
    if connected is not True:
        reasons.append('No hay conexión MAVLink reciente')
    if armed is not False:
        reasons.append('El ROV debe permanecer desarmado')
    if output_enabled is not False:
        reasons.append('La salida ROS de propulsión debe estar deshabilitada')
    if camera_required and not camera_available:
        reasons.append('Esta sesión exige video reciente, pero no está disponible')
    return not reasons, reasons
