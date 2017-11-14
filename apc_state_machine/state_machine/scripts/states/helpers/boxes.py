import json_functions

boxes = {}
order = {}
wanted_items = []

def load_data():
    global boxes
    global order
    global wanted_items

    box_sizes = json_functions.load_file('pick', 'box_sizes.json')
    order = json_functions.load_file('pick', 'order_file.json')['orders']

    box_list = []
    total_x = 0
    for o in order:
        bid = o['size_id']
        dims = []
        for b in box_sizes['boxes']:
            if b['size_id'] == bid:
                dims = b['dimensions']
                break
        wanted_items.extend(o['contents'])

        sort_by = tuple(dims[:2])  # Sort by the first two dimensions
        if dims[0] < 0.37:
            # if the box fits with long edge along y,
            #  then swap x and y
            dims = [dims[1], dims[0], dims[2]]

        total_x += dims[0]

        box_list.append({
            'size_id': bid,
            'dimensions': dims,
            'sort_by': sort_by
        })

    # sort the boxes from largest to smallest
    box_list.sort(key=lambda x: x['sort_by'], reverse=True)

    # x pos of largest box
    MAX_X = 1.16
    MIN_X = MAX_X - 1.3
    MIN_Y = 0.71
    #x = MIN_X + (MAX_X-MIN_X)/2 + total_x/2
    x = 0.978  # It's not set as hard limit.

    for b in box_list:
        b['top'] = MIN_Y + b['dimensions'][1]
        b['left'] = x
        b['bottom'] = MIN_Y
        b['right'] = x - b['dimensions'][0]

        x -= b['dimensions'][0] + 0.01  # 2cm buffer for boxes.
        boxes[b['size_id']] = b

load_data()
