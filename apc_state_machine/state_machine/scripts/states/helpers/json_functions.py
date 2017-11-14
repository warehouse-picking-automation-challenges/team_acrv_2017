import json

BASE_PATH = '/home/apc-cameras/APC_JSON_DATA/'

def load_file(task, filename):
    with open(BASE_PATH + task + '/' + filename) as f:
        return json.load(f)

def save_file(task, filename, data):
    with open(BASE_PATH + task + '/' + filename, 'w') as f:
        json.dump(data, f, indent=4, separators=(',', ': '))
