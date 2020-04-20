BEGIN;

  CREATE TABLE object3d(
    name VARCHAR(256) PRIMARY KEY,
    objtype INTEGER,
    data TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT NOW()
  );

  CREATE TABLE object3d_registry(
    name VARCHAR(256) PRIMARY KEY,
    object3d_name VARCHAR(256),
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    CONSTRAINT fk_object3d_name FOREIGN KEY (object3d_name) REFERENCES object3d (name) ON DELETE CASCADE
  );

COMMIT;
