"""Pure presentation rules for the supervised sequence dashboard."""


STAGES = ('configure', 'prepare', 'arm', 'guided', 'home', 'execute')


def controller_tuning_payload(
    xy_tolerance,
    recovery_radius,
    recovery_speed,
    recovery_min_speed,
    recovery_delay,
    position_gain,
    velocity_damping,
):
    """Return the controller fields understood by the physical executor."""
    return {
        'xy_tolerance': xy_tolerance,
        'recovery_radius': recovery_radius,
        'recovery_speed': recovery_speed,
        'recovery_min_speed': recovery_min_speed,
        'recovery_delay': recovery_delay,
        'position_gain': position_gain,
        'velocity_damping': velocity_damping,
    }


def workflow_states(status, fresh, has_steps, plan_matches):
    """Return visual states without changing controller decisions."""
    states = {name: 'neutral' for name in STAGES}
    if not has_steps:
        states['configure'] = 'active'
        return states
    states['configure'] = 'done'
    if not fresh or status is None:
        states['prepare'] = 'blocked'
        return states

    prepared = plan_matches and status.get('validated', False)
    if not prepared:
        states['prepare'] = (
            'active'
            if status.get('armed') is False and status.get('mode') == 'MANUAL'
            else 'blocked'
        )
        return states
    states['prepare'] = 'done'

    if status.get('armed') is not True:
        states['arm'] = 'active'
        return states
    states['arm'] = 'done'

    if status.get('mode') != 'GUIDED':
        states['guided'] = (
            'active'
            if status.get('navigation_ready') is True and not status.get('blockers')
            else 'blocked'
        )
        return states
    states['guided'] = 'done'

    if not status.get('home_ready', bool(status.get('home'))):
        states['home'] = (
            'active'
            if status.get('navigation_ready') is True and not status.get('blockers')
            else 'blocked'
        )
        return states
    states['home'] = 'done'

    phase_code = (status.get('phase') or {}).get('code')
    if phase_code == 'COMPLETADO':
        states['execute'] = 'done'
    elif status.get('state') == 'RUNNING':
        states['execute'] = 'active'
    else:
        states['execute'] = (
            'active'
            if status.get('enabled') is True
            and status.get('navigation_ready') is True
            and not status.get('blockers')
            else 'blocked'
        )
    return states


def operator_guidance(status, fresh, has_steps, plan_matches):
    """Describe the next safe operator action and the reasons blocking it."""
    if not fresh or status is None:
        return {
            'message': 'Esperando ejecutor y puente',
            'state': 'warning',
            'reasons': ['No hay un estado reciente del ejecutor.'],
            'abort_available': False,
        }
    if status.get('state') == 'RUNNING':
        return {
            'message': 'Secuencia en ejecución · supervisa el movimiento',
            'state': 'info',
            'reasons': ['El aborto hacia MANUAL permanece disponible.'],
            'abort_available': True,
        }
    if not has_steps:
        return {
            'message': 'Siguiente: añade al menos una orden',
            'state': 'warning',
            'reasons': ['El plan está vacío.'],
            'abort_available': True,
        }
    if not plan_matches or not status.get('validated', False):
        if status.get('armed') is False and status.get('mode') == 'MANUAL':
            message = 'Siguiente: 2 · preparar y validar el plan'
            reasons = ['La configuración actual todavía no fue confirmada.']
        else:
            message = 'Preparación bloqueada'
            reasons = ['Para preparar: modo MANUAL y vehículo desarmado.']
        return {
            'message': message,
            'state': 'warning',
            'reasons': reasons,
            'abort_available': True,
        }
    if status.get('armed') is not True:
        return {
            'message': 'Siguiente: 3 · armar deliberadamente desde el mando',
            'state': 'info',
            'reasons': ['Esta interfaz no arma el vehículo.'],
            'abort_available': True,
        }
    blockers = list(status.get('blockers') or [])
    if status.get('navigation_ready') is not True or blockers:
        if status.get('navigation_ready') is not True:
            blockers.insert(0, 'La navegación todavía no está lista.')
        waiting_for_home = status.get('mode') == 'GUIDED' and not status.get(
            'home_ready', bool(status.get('home'))
        )
        return {
            'message': (
                'Esperando GUIDED estable para fijar Home'
                if waiting_for_home
                else 'GUIDED bloqueado · corrige las condiciones indicadas'
            ),
            'state': 'warning',
            'reasons': blockers,
            'abort_available': True,
        }
    if status.get('mode') != 'GUIDED':
        return {
            'message': 'Siguiente: 4 · solicitar GUIDED seguro',
            'state': 'info',
            'reasons': ['El plan está preparado, armado y con navegación válida.'],
            'abort_available': True,
        }
    if not status.get('home_ready', bool(status.get('home'))):
        return {
            'message': 'Siguiente: 5 · fijar Home con la pose retenida por GUIDED',
            'state': 'info',
            'reasons': [
                'X=0, Y=0 y rumbo 0° usarán la referencia capturada al entrar en GUIDED.'
            ],
            'abort_available': True,
        }
    if status.get('enabled') is not True:
        return {
            'message': 'GUIDED listo · salida física deshabilitada',
            'state': 'warning',
            'reasons': ['Este lanzamiento no permite ejecutar movimiento real.'],
            'abort_available': True,
        }
    return {
        'message': 'Siguiente: 6 · iniciar el plan preparado',
        'state': 'success',
        'reasons': ['Todas las condiciones de inicio están confirmadas.'],
        'abort_available': True,
    }
