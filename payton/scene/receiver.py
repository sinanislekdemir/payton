"""Receiver module to overcome internal circular dependency on Scene feedbacks
"""


class Receiver(object):
    """Receiver simply does nothing.
    It is a base meta class to let objects to receive feedbacks or back-calls
    from their childs without getting any circlular references.
    """

    def __str__(self) -> str:
        return "Scene"
