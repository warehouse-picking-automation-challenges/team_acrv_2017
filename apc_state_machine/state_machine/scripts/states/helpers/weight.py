from item_meta import item_meta

def weight_matches(item_name, measured_weight, possible_objects, max_objects):
    expected_weight = item_meta[item_name].get('weight', 0.0) * 1000.0  # scales are grams, meta is kilos
    if expected_weight == 0:
        rospy.logerr('Item %s doesn\'t have a weight associated with it' % item_name)
        return True, []
    max_deviation = min(max(6, expected_weight * 0.05), 25)  # Percentage, as long as it's betwen the limits

    possible_matches = []
    for o_name in item_meta:
        if 'weight' not in item_meta[o_name]:
            continue
        if o_name in possible_objects:
            dw = abs(measured_weight - item_meta[o_name]['weight'] * 1000.0)
            if dw < max_deviation:
                possible_matches.append( (dw, o_name) )

    possible_matches.sort()
    possible_matches = possible_matches[:max_objects]
    for m in possible_matches[:max_objects]:
        if m[1] == item_name:
            return True, possible_matches

    return False, possible_matches
