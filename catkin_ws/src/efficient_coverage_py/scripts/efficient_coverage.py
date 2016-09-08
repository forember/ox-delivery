#!/usr/bin/env python

import rospy, os

def parse_tours(tour_file):
    import json
    tours = []
    current_tour = None
    for line in tour_file:
        line = line.strip()
        if line.startswith('Start'):
            current_tour = []
        elif line.startswith('End'):
            tours.append(current_tour)
            current_tour = None
        elif line:
            line = line.replace(') (', '), (')
            line = '[' + line + ']'
            line = line.replace('(', '[').replace(')', ']')
            current_tour.extend(json.loads(line))
    return tours

def generate_tours(input_dir, input_name, robot_count=1):
    import tempfile, subprocess, shutil
    tmpdir = tempfile.mkdtemp()
    tours = None
    try:
        cmd_spec = ['/usr/bin/env', 'afrl-oxdel',
            os.path.realpath(input_dir), input_name, robot_count]
        print('Calling {}\n  in {}:'.format(cmd_spec, tmpdir))
        subprocess.call(cmd_spec, cwd=tmpdir)
        tour_file_path = os.path.join(tmpdir, 'tourLines.txt')
        with open(tour_file_path) as tour_file:
            tours = parse_tours(tour_file)
    finally:
        shutil.rmtree(tmpdir)
    return tours

def main():
    argv = rospy.myargv()
    if len(argv) != 5:
        print(('Usage: {} <input dir> <input name> <total robot #>'
            ' <this robot #>').format(argv[0]))
        return 1
    robot_id = int(argv[4])
    tour = generate_tours(*argv[1:4])[robot_id]
    print('{}: TOUR:\n  {}'.format(argv[0], tour))

if __name__ == '__main__':
    raise SystemExit(main())
