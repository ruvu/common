import numpy as np


class ProcrustesGroupPositioner(object):

    def __init__(self, tag_locations, **kwargs):
        self.tag_locations = tag_locations
        self.max_cost = kwargs.get('max_cost', 1000)
        self.min_tag_positions = kwargs.get('min_tag_positions', 2)
        self.procrustes_dim = kwargs.get('procrustes_dim', 2)
        self.coordinates = None
        self.orientation = None

    def calculate_group_position(self, positioning_input):
        if len(positioning_input) < 2:
            return {
                'success': False,
                'error_code': 'NOT_ENOUGH_POSITIONS',
            }
        # Initialize corrupted positions list
        corrupted_list = [tag_id for tag_id in positioning_input if not positioning_input[tag_id]['success']]
        for loop_count in range(len(positioning_input)):
            # uncorrupted tags to be used for positioning
            positioning_tags = [t_id for t_id in positioning_input
                                if t_id in self.tag_locations and t_id not in corrupted_list]
            if len(positioning_tags) < 2:
                return {
                    'success': False,
                    'error_code': 'NOT_ENOUGH_NON_CORRUPT_POSITIONS',
                }
            # procrustes mathematics
            c = ['x', 'y', 'z']
            translation = [np.mean([positioning_input[t_id]['coordinates'][c[i]] - self.tag_locations[t_id][i]
                                    for t_id in positioning_tags])
                           for i in range(3)]
            a_mat = np.array([[self.tag_locations[t_id][i] for t_id in positioning_tags]
                              for i in range(self.procrustes_dim)])
            b_mat = np.array([[positioning_input[t_id]['coordinates'][c[i]] - translation[i]
                               for t_id in positioning_tags]
                              for i in range(self.procrustes_dim)])
            m_mat = np.dot(b_mat, np.transpose(a_mat))
            u, s, vh = np.linalg.svd(m_mat)
            rotation = np.dot(u, vh)
            # Check for corrupted positions and remove them
            e_mat = np.dot(rotation, a_mat) - b_mat
            new_corrupted = []
            for i in range(e_mat.shape[1]):
                if np.linalg.norm(e_mat[:, i]) > self.max_cost:
                    new_corrupted.append(positioning_tags[i])
            if len(new_corrupted) == 0:
                break
            corrupted_list.extend(new_corrupted)

        self.coordinates = {"x": translation[0], "y": translation[1], "z": translation[2]}
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        if self.procrustes_dim > 2:
            pitch = np.arctan2(-rotation[2, 0],
                               np.sqrt(rotation[2, 1]**2 + rotation[2, 2]**2))
            roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        else:
            pitch = 0
            roll = 0
        self.orientation = {'yaw': yaw, 'pitch': pitch, 'roll': roll}

        return {
            'success': True,
            'coordinates': self.coordinates,
            'orientation': self.orientation
        }
