BEGIN;
  INSERT INTO object3d (id, objtype, name, data) VALUES (1, 1, 'test_obj', 'test data') ON CONFLICT DO NOTHING;
  INSERT INTO object3d (id, objtype, name, data) VALUES (2, 2, 'testbox_sdf', '<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="Box">
  <static>true</static>
    <link name="BoxLink">
      <pose>0 0 0 0 0 0</pose>
      <visual name="BoxViz">
        <geometry>
          <box><size>0.5 0.5 0.5</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>') ON CONFLICT DO NOTHING;
  INSERT INTO object3d_registry (name, object3d_id) VALUES ('reg_test_obj', 1) ON CONFLICT DO NOTHING;
  INSERT INTO object3d_registry (name, object3d_id) VALUES ('reg2_test_obj', 1) ON CONFLICT DO NOTHING;
  INSERT INTO object3d_registry (name, object3d_id) VALUES ('testbox', 2);
COMMIT;
