"""Format a SceneObservation into a short string for the LLM."""

from perception.scene_parsing import SceneObservation


def format_observation(obs: SceneObservation) -> str:
    """Turn a SceneObservation into the string the LLM reads.

    Strings use the bucket vocabulary verbatim so the LLM, the system
    prompt, and the perception layer share words.
    """
    if not obs.found:
        return "target not found"

    direction = obs.direction or "unknown direction"
    distance = (
        f"{obs.distance} distance" if obs.distance is not None
        else "unknown distance"
    )

    base = f"target found at {direction}, {distance}"
    if obs.should_stop:
        return f"{base} (arrived: call stop)"
    return base
