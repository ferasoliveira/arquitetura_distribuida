# conftest.py — Configuração global de testes para o Passo 3
# Mock do módulo 'controller' do Webots para permitir importação de hil_bridge.py
# fora do ambiente Webots.

import sys
from unittest.mock import MagicMock

# Instala o mock antes de qualquer import de hil_bridge nos testes.
# O módulo é registrado sob o nome exato que hil_bridge.py usa no import.
if "controller" not in sys.modules:
    _ctrl = MagicMock()
    _ctrl.Robot = MagicMock
    _ctrl.Supervisor = MagicMock
    sys.modules["controller"] = _ctrl
