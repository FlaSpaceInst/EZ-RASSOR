"""Process keyboard events."""
import itertools


def leader_keys(events):
    """Generate leader keys from a stream of events.

    A "leader key" is the most recently-pressed key that is still pressed
    after any given event, or None if no key is pressed.

    Example key event sequence and corresponding generated sequence:
        press "a"     :   yield "a"
        press "b"     :   yield "b"
        release "b"   :   yield "a"
        press "c"     :   yield "c"
        release "a"   :
        release "c"   :   yield None

    The point of this generator is to filter out repeated events for keys that
    remain pressed for a long time, and also maintain state when multiple keys
    are pressed and held in a sequence.
    """
    pressed = []
    for event in events:
        if event.pressed:
            if event.key not in pressed:
                pressed.append(event.key)
                yield event.key
            else:
                pressed = [key for key in pressed if key != event.key]
                pressed.append(event.key)
        else:
            pressed = [key for key in pressed if key != event.key]
            if pressed:
                yield pressed[-1]
            else:
                yield None


def handle_keypresses(handlers, events):
    """Process keypresses using a map ("handlers") of keys to functions."""
    relevant_events = itertools.ifilter(
        lambda event: event.key in handlers.keys(), events,
    )

    for key in leader_keys(relevant_events):
        handlers[key]()
