def monitor_following_distance(distances: list[float], speeds: list[float]) -> tuple[int, float, int]:
    """
    Analyzes following distance compared to safe distance (speed * 0.5).
    
    Args:
        distances (list[float]): Distance to the lead car at each second.
        speeds (list[float]): Speed of our car at each second.
        
    Returns:
        tuple[int, float, int]: (tailgating_seconds, minimum_distance, tailgate_incidents)
            - tailgating_seconds: total seconds distance was < safe distance
            - minimum_distance: absolute closest distance to the lead car (return 0.0 if empty list)
            - tailgate_incidents: number of separate instances the car started tailgating
    """
    
    safe_distance = 0
    tailgating_seconds = 0
    previous_tailgating = False
    tailgate_incidents= 0
    if len(distances)==0:
        minimum_distance=0.0
    else:
        minimum_distance=min(distances)

    for i in range(len(speeds)):
        safe_distance = speeds[i] * 0.5

        if distances[i] < safe_distance:
            tailgating_seconds += 1
            current_tailgating = True

            if previous_tailgating == False:
                tailgate_incidents += 1
        else:
            current_tailgating = False

        previous_tailgating = current_tailgating

    return (tailgating_seconds, minimum_distance, tailgate_incidents)
