BEGIN;

  CREATE TABLE object3d(
    id SERIAL PRIMARY KEY,
    objtype INTEGER,
    name VARCHAR(256),
    data TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT NOW()
  );

  CREATE TABLE object3d_registry(
    name VARCHAR(256) PRIMARY KEY,
    object3d_id INTEGER,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    CONSTRAINT fk_object3d_id FOREIGN KEY (object3d_id) REFERENCES object3d (id) ON DELETE CASCADE
  );

COMMIT;
