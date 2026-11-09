"""Transport helpers for the BlueOS-resident sequence executor."""

import copy
import time
from urllib.parse import urlparse
import uuid


def normalise_agent_url(value):
    """Return a safe HTTP base URL without a trailing slash."""
    url = str(value).strip().rstrip('/')
    parsed = urlparse(url)
    if parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError('La URL del agente debe ser HTTP y no contener credenciales')
    if parsed.path or parsed.params or parsed.query or parsed.fragment:
        raise ValueError('La URL del agente no debe incluir ruta, consulta ni fragmento')
    port = parsed.port or 80
    if not 1 <= port <= 65535:
        raise ValueError('Puerto del agente fuera de rango')
    return url


def initial_offline_status(session=None):
    """Create a complete status packet while the onboard agent is unavailable."""
    return {
        'session': session or ('proxy-' + uuid.uuid4().hex),
        'ticket': uuid.uuid4().hex,
        'state': 'IDLE',
        'note': 'Esperando agente de secuencias en BlueOS',
        'nav_error': 'Agente embarcado sin conexión',
        'mode': None,
        'armed': None,
        'connected': False,
        'enabled': False,
        'blockers': ['Agente embarcado sin conexión'],
        'surface': False,
        'home': None,
        'home_ready': False,
        'pose': None,
        'validated': False,
        'prepared_by': None,
        'returning': False,
        'plan_mode': 'absolute',
        'step': 0,
        'total': 0,
        'goal': None,
        'navigation_ready': False,
        'phase': {
            'code': 'BLOQUEADO',
            'title': 'AGENTE BLUEOS SIN CONEXIÓN',
            'detail': 'No se reciben estados del ejecutor embarcado',
        },
        'agent_link': False,
        'execution_host': 'blueos',
    }


def disconnected_status(previous, reason, now=None):
    """Mark a cached state disconnected without pretending the mission stopped."""
    status = copy.deepcopy(previous) if isinstance(previous, dict) else initial_offline_status()
    status['ticket'] = uuid.uuid4().hex
    status['connected'] = False
    status['agent_link'] = False
    status['agent_link_error'] = str(reason)
    status['agent_link_lost_at'] = float(time.monotonic() if now is None else now)
    blockers = [str(item) for item in status.get('blockers', [])]
    marker = 'Sin enlace con el agente embarcado'
    if marker not in blockers:
        blockers.append(marker)
    status['blockers'] = blockers
    status['navigation_ready'] = False
    status['phase'] = {
        'code': 'PAUSA' if status.get('state') == 'RUNNING' else 'BLOQUEADO',
        'title': 'ENLACE CON BLUEOS INTERRUMPIDO',
        'detail': (
            'El agente conserva la autoridad local; la interfaz se resincronizará '
            'cuando vuelva la red.'
        ),
    }
    return status
