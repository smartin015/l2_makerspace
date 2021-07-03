function getCenterPoint(mesh) {
    var geometry = mesh.geometry;
    geometry.computeBoundingBox();
    var center = new THREE.Vector3();
    geometry.boundingBox.getCenter( center );
    mesh.localToWorld( center );
    return center;
}

function attachLoaded(scene, camera, controls, renderer, geometry) {
    var material = new THREE.MeshPhongMaterial({ 
        color: 0xff5533, 
        opacity: 0.5,
        transparent: true,
        specular: 100, 
        shininess: 100 });
    var mesh = new THREE.Mesh(geometry, material);
    mesh.position = getCenterPoint(mesh);
    scene.add(mesh);

    var middle = new THREE.Vector3();
    geometry.computeBoundingBox();
    geometry.boundingBox.getCenter(middle);
    mesh.geometry.applyMatrix(new THREE.Matrix4().makeTranslation( 
                                  -middle.x, -middle.y, -middle.z ) );
    var largestDimension = Math.max(geometry.boundingBox.max.x,
                                geometry.boundingBox.max.y, 
                                geometry.boundingBox.max.z)
    camera.position.z = largestDimension * 2.5;



    //var origin = new THREE.Vector3(0,0,0);
    //var up = new THREE.Vector3(0,0,1);
    //var rcs = raycastCylinder(origin, up, 50, 10, 200, 10);
    //var tcs = raycastCylinder(origin, up, 45, 10, 200, 10);
    //scene.add(rcs);
    //scene.add(tcs);

    var clearance_radius = clearanceRadius(null);
    var num_rad = 10;
    var num_h = 10;
    const pgeom = findSurfacePoints(num_rad, num_h, 10, mesh);
    console.log(pgeom.attributes.position);
    const pm = new THREE.PointsMaterial( {color: 0x0000ff} );
    scene.add(new THREE.Points(pgeom, pm));

    const segments = floodFillSegments(clearance_radius, num_rad, num_h, pgeom);
    console.log(segments);

    var animate = function () {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }; 

    animate();
}

function STLViewer(model, elementID) {
    var elem = document.getElementById(elementID)
    var camera = new THREE.PerspectiveCamera(70, 
        elem.clientWidth/elem.clientHeight, 1, 1000);
    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(elem.clientWidth, elem.clientHeight);
    elem.appendChild(renderer.domElement);
    window.addEventListener('resize', function () {
        renderer.setSize(elem.clientWidth, elem.clientHeight);
        camera.aspect = elem.clientWidth/elem.clientHeight;
        camera.updateProjectionMatrix();
    }, false);
    var controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.rotateSpeed = 0.05;
    controls.dampingFactor = 0.1;
    controls.enableZoom = true;
    controls.autoRotate = true;
    controls.autoRotateSpeed = .75;

    var scene = new THREE.Scene();
    scene.add(new THREE.HemisphereLight(0xffffff, 1.5));

    // (new THREE.STLLoader()).load(model, (geometry) => attachLoaded(scene, geometry));
    attachLoaded(scene, camera, controls, renderer, new THREE.CylinderGeometry(5, 5, 10, 10, 10));
}
