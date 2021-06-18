

function raycastCylinder(position, upaxis, radius, length, radseg, heightseg) {
  const geometry = new THREE.CylinderGeometry(radius, radius, length, radseg, heightseg, true);
  const material = new THREE.PointsMaterial( {color: 0x00ff00, size: 0.4} );
  const cylinder = new THREE.Points( geometry, material );
	cylinder.position = position
	cylinder.up = upaxis
	return cylinder;
}

function findSurfacePoints(radsamp, lensamp, length, mesh) {
  const raycaster = new THREE.Raycaster();
  const vertices = [];
  const R = 1000.0;
  let misses = 0;
  for (let y = -(length/2); y < length/2; y += lensamp/length) {
    for (let r = 0; r < radsamp; r++) {
      let pos = new THREE.Vector3(R * Math.cos(r/radsamp*2*Math.PI), 
                                  y,
                                  R * Math.sin(r/radsamp*2*Math.PI));
      let dir = pos.clone().negate().setY(0).normalize(); // Y is up
      raycaster.set(pos, dir);
      const result = raycaster.intersectObject(mesh);
      if (result.length > 0) {
        vertices.push(result[0].point.x);
        vertices.push(result[0].point.y);
        vertices.push(result[0].point.z);
      } else {
        misses++;
      }
    }
  }
  console.log(`Missed ${misses} raycasts (of ${radsamp*lensamp}, ${parseInt(100*misses/(radsamp*lensamp))}%)`);
  const geometry = new THREE.BufferGeometry();
  // itemSize = 3 because there are 3 values (components) per vertex
  geometry.setAttribute( 'position', new THREE.BufferAttribute( new Float32Array(vertices), 3 ) );
  return geometry;
}
