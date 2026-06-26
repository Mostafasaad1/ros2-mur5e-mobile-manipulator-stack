import os
import yaml
import pytest

def test_sensors_3d_yaml_exists_and_valid():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'sensors_3d.yaml'
    )
    assert os.path.exists(config_path), "sensors_3d.yaml does not exist"
    
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
        
    assert 'sensors' in data
    assert len(data['sensors']) > 0
    sensor_name = data['sensors'][0]
    sensor = data[sensor_name]
    assert sensor['sensor_plugin'] == 'occupancy_map_monitor/PointCloudOctomapUpdater'
    assert sensor['point_cloud_topic'] == '/camera/points'
