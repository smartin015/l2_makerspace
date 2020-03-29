BEGIN;
  INSERT INTO object3d (objtype, name, data) VALUES (1, 'test_obj', 'test data');
  INSERT INTO object3d_registry (name, object3d_id) VALUES ('reg_test_obj', 1);
  INSERT INTO object3d_registry (name, object3d_id) VALUES ('reg2_test_obj', 1);
COMMIT;
