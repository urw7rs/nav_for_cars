# navigation for ackermann drive robots (cars)

run `roslaunch nav_for_cars move.launch path_file_name:=path` to load path.txt file in `nav_for_cars/path` add `path_file_dir:=/abs/path/to/dir` to modify search path to /abs/path/to/dir

currently loads global path given in path file and uses a stanley controller to follow loaded path

`path.txt -> load_path.py -> /path -> follow_path.py -> /ackermann_cmd`
