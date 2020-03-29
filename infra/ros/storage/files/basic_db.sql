BEGIN;

CREATE TABLE object3d(
  id INTEGER PRIMARY KEY,
  objtype INTEGER,
	name VARCHAR(256),
	data TEXT,
	created_at TIMESTAMP NOT NULL
);

CREATE TABLE object3d_registry(
  id VARCHAR(256) PRIMARY KEY,
  object3d_id INTEGER,
  created_at TIMESTAMP NOT NULL,
  -- CONSTRAINT fk_object3d_id FOREIGN KEY (object3d_id) REFERENCES object3d (id) ON DELETE CASCADE
);

CREATE TABLE users(
  id INTEGER PRIMARY KEY
  created_at TIMESTAMP NOT NULL,
);

CREATE TABLE projects(
  id INTEGER PRIMARY KEY,
  name VARCHAR(256),
  owner_id INTEGER,
  repository_url TEXT,
  created_at TIMESTAMP NOT NULL,
  -- CONSTRAINT fk_owner_id FOREIGN KEY (owner_id) REFERENCES users (id) ON DELETE CASCADE
);

CREATE TABLE items(
  id INTEGER PRIMARY KEY,
  project_id INTEGER,
  content TEXT,
  created_at TIMESTAMP NOT NULL,
  -- CONSTRAINT fk_project_id FOREIGN KEY (project_id) REFERENCES projects (id) ON DELETE CASCADE
);

COMMIT;
