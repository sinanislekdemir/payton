"""Receiver module to overcome internal circular dependency on Scene feedbacks
"""


class Receiver(object):
    """Receiver simply does nothing.
    It is a base meta class to let objects receive feedbacks or callbacks
    from their children without getting any circular references.
    """

    def __str__(self) -> str:
        return "Scene"
