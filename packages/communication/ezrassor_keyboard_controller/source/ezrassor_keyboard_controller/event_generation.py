"""Generate keyboard events."""
import Queue


class Event:
    """A keyboard event."""

    def __init__(self, key, pressed):
        """Create a new keyboard event with a key and pressed status."""
        self.key = key
        self.pressed = pressed


def pynput_events(halt_queue):
    """Generate keyboard events with the pynput library.

    This generator terminates once anything is pushed into the halt_queue.
    """
    try:
        import pynput
    except ImportError:
        raise ImportError(
            'dependency "pynput" required by this function but not installed',
        )

    with pynput.keyboard.Events() as events:
        while halt_queue.empty():
            try:
                event = events.get(1.0)
            except Queue.Empty:
                continue

            if event is None:
                continue
            elif isinstance(event.key, pynput.keyboard.Key):
                yield Event(
                    event.key.name,
                    isinstance(event, pynput.keyboard.Events.Press),
                )
            elif isinstance(event.key, pynput.keyboard.KeyCode):
                yield Event(
                    event.key.char,
                    isinstance(event, pynput.keyboard.Events.Press),
                )
