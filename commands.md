
Run particle filter:
`
ros2 launch robot_localization test_pf.py map_yaml:=maps/gauntlet.yaml
`

Run amcl:
`
ros2 launch robot_localization test_amcl.py map_yaml:=maps/gauntlet.yaml

`

Set Fixed Frame to Map
Change particlecloud Reliabilty to Best Effort
Change map Durability to Transient Local


