"""
# Payton Library

.. include:: ../README.md
"""

import OpenGL

# OpenGL Error Checking has a great over-head to render cycle
OpenGL.ERROR_CHECKING = False
OpenGL.ERROR_LOGGING = False

from payton.ai import catalog as ai_catalog
from payton.ai import context as ai_context

__all__ = ["ai_catalog", "ai_context"]
