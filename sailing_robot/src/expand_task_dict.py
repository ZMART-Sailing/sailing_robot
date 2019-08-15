import LatLon


def expand_id(wp_params):
    if 'table' not in wp_params:
        return wp_params
    coordinates = wp_params['table']
    if 'tasks' in wp_params:
        # Long specification: list of tasks
        for wp_task in wp_params['tasks']:
            kind = wp_task['kind']
            if kind == 'to_waypoint' or kind == 'keep_station':
                if isinstance(wp_task['waypoint'], str):
                    wp_task['waypoint_id'] = wp_task['waypoint']
                    wp_task['waypoint'] = coordinates[wp_task['waypoint']]
            else:
                continue
    else:
        # Short specification: just a series of waypoints to go around
        if isinstance(wp_params['list'][0], str):
            wp_params['id_list'] = wp_params['list']
            wp_params['list'] = [coordinates[wpid] for wpid in wp_params['list']]

    return wp_params


def expand_task_dict(wp_params):
    wp_params = expand_id(wp_params)
    target_radius = wp_params['target_radius']
    tack_voting_radius = wp_params['tack_voting_radius']

    def expand_to_waypoint(wp, wpid = ''):
        return {
            'kind': 'to_waypoint',
            'waypoint': LatLon.LatLon(*wp),
            'waypoint_id': wpid,
            'target_radius': target_radius,
            'tack_voting_radius': tack_voting_radius
        }

    res = []
    if 'tasks' in wp_params:
        # Long specification: list of tasks
        for wp_task in wp_params['tasks']:
            kind = wp_task['kind']
            if kind == 'to_waypoint':
                expanded_task = {
                    'kind': 'to_waypoint',
                    'waypoint': LatLon.LatLon(*wp_task['waypoint']),
                    'waypoint_id': wp_task['waypoint_id'],
                    'target_radius': wp_task.get('accept_radius', target_radius),
                    'tack_voting_radius': wp_task.get('accept_radius', tack_voting_radius)
                }
            elif kind == 'keep_station':
                expanded_task = {
                    'kind': 'keep_station',
                    'marker': LatLon.LatLon(*wp_task['waypoint']),
                    'marker_id': wp_task['waypoint_id'],
                    'linger': wp_task.get('linger', 300),
                    'radius': wp_task.get('radius', 5),
                    'threshold': wp_task.get('threshold', 10),
                    'accept_radius': wp_task.get('radius', 10),
                    'tack_voting_radius': wp_task.get('accept_radius', tack_voting_radius),
                    'target_radius': wp_task.get('accept_radius', target_radius)
                }
            elif kind == 'start_timer':
                expanded_task = wp_task.copy()
            else:
                continue

            # Copy over any jump label
            expanded_task['jump_label'] = wp_task.get('jump_label', None)
            res.append(expanded_task)
    else:
        # Short specification: just a series of waypoints to go around
        if 'id_list' not in wp_params:
            wp_params['id_list'] = [''] * len(wp_params['list'])
        for wp, wpid in zip(wp_params['list'], wp_params['id_list']):
            res.append(expand_to_waypoint(wp, wpid))

    return res
