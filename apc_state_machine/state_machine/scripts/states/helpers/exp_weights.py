from item_meta import item_meta

with open('/home/apc-cameras/weights.txt', 'w') as f:
    for name in item_meta.keys():
        print(name)
        f.write(name + '\t' + str(item_meta[name].get('weight', 0)) + '\n')
