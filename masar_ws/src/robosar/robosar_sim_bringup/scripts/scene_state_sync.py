"""Pure helpers for synchronizing configured scene records with Gazebo state."""


def transport_pose_updates(model_names, xy_positions, record_count):
    """Map exact transport-box model names to bounded record indices."""
    updates = {}
    prefix = "robosar_transport_box_"
    for name, position in zip(model_names, xy_positions):
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix) :]
        if not suffix.isdigit():
            continue
        index = int(suffix)
        if index < 0 or index >= int(record_count):
            continue
        updates[index] = (float(position[0]), float(position[1]))
    return updates
