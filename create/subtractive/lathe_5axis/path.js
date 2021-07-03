class Mat {
  constructor(w,h) {
    this.h=h;
    this.w=w;
    this.m=Array.from(Array(w), ()=>new Array(h));
  }
  at(uv) {
    return this.m[uv[0]][uv[1]];
  }
  set(uv, v) {
    try {
      this.m[uv[0]][uv[1]] = v;
    } catch (e) {
      throw `Failed to set matrix ${uv} to ${v}: ${e}`;
    }
  }
}

function uvProjectCylinder(target_geometry, num_rad, num_h) {
  // Unwrap a cylindrical view of target points into a dense UV matrix (indices map to discretized theta, height)
  let pts = target_geometry.attributes.position;
  console.log(pts);
  if (num_rad*num_h != pts.length / 3) {
    throw `Target geometry has ${pts.length/3} vertices, expected ${num_rad}*${num_h}=${num_rad*num_h} for UV unwrap`;
  }

  // Get V bounds - note that U bounds are implied [0,2*pi)
  let min_y = 999999;
  let max_y = -999999;
  for (let i = 0; i+2 < pts.count; i += 3) {
    min_y = Math.min(min_y, pts.array[i+1]);
    max_y = Math.max(max_y, pts.array[i+1]);
  }
  let du = 2*Math.PI/num_rad;
  let dv = (max_y-min_y)/num_h;

  console.log(`min_y ${min_y} max_y ${max_y}`);

  let uv = new Mat(num_rad, num_h);
  for (let i = 0; i < pts.count; i += 3) {
    // Get UV coordinates from projection
    console.log(pts.array[i], pts.array[i+1], pts.array[i+2]);
    let u = Math.round(Math.PI + Math.atan2(pts.array[i], pts.array[i+2]) / du);
    let v = Math.round(pts.array[i+1]/dv - min_y);
    // Cell value is the radial distance from the Y axis
    uv.set([u,v], Math.sqrt(pts.array[i]*pts.array[i] + pts.array[i+2]*pts.array[i+2]));
  }
  return uv;
}

function sortWithIndices(uv) {
  // Returns a list of indices starting with the smallest UV value
  // https://stackoverflow.com/questions/3730510/javascript-sort-array-and-return-an-array-of-indices-that-indicates-the-positio
  var test_with_index = [];
  for (let u of uv) {
    for (let v of uv) {
      test_with_index.push([uv.at([u, v]), [u, v]]);
    }
  }
  test_with_index.sort(function(left, right) {
    return left[0] < right[0] ? -1 : 1;
  });
  var indexes = [];
  for (var j in test_with_index) {
      indexes.push(test_with_index[j][1]);
  }
  return indexes
}

function clearanceRadius(stock_geometry) {
  return 20; // TODO max stock_geometry radius plus some extra
}

function cylinderCutPath(name, clearance_radius, num_rad, num_h, stepdown, target_geometry) {
  // Generates and returns GCode to cut a cylindrical "lathe" path 
  // around a part.
  // INVARIANT: target_points are ordered cylindrically, top-to-bottom
  let result = "; Begin ";
  
  let done = false;

  let segments = floodFillSegments(clearance_radius, num_rad, num_h, target_geometry)

  // At this point, we have a list of cuttable segments including valid starting positions above the lowest
  // point in that segment, and the maximum height of that segment that is uncut.
  console.log(segments);

  // Add the "facing" pass to get us to total_max_height so we can cut with impunity.
  result += `\n; TODO facing pass down to ${total_max_height} radius`;

  for (let i = 0; i < segments.length; i++) {
    let seg = segments[i];
    let cur_height = seg[1];
    // TODO start at seg[0] and work outwards, hitting all points where visited == i
    // and uv.at(seg[0]) <= cur_height
    // then subtract cur_height by stepdown and repeat
    result += `\n; TODO cut segment at UV ${seg[0]} from height ${seg[1]}`;
  }

  return result;
}

function floodFillSegments(clearance_radius, num_rad, num_h, target_geometry) {
  let rad = clearance_radius; // Start out outside the bounds of the material
  let uv = uvProjectCylinder(target_geometry, num_rad, num_h);
  sorted_idxs = sortWithIndices(uv)
  let candidates = [[sorted_idxs[0], uv[sorted_idxs[0][0]][sorted_idxs[0][1]]]]; // Queue, sorted by height
  let sort_i = 1;
  let segments = []; // contains [[u,v],max_height], which should be enough to prevent double-cutting
  let seg_i = 0; // Fill the visited matrix with the segment index
  let visited = new Mat(num_rad, num_h);
  let cur_max_height = 0;
  let total_max_height = 0; // Max of cur_max_height values
  // Modified flood-fill - eat volume segments from deepest point outwards to find contiguous areas
  // that we can mill in one go.
  while (sort_i < sorted_idxs.length) {
    // Check all target points to see if we're inside
    if (candidates.length == 0) {
      sort_i++;
      if (!visited.at(sorted_idxs[sort_i])) {
        segments.push(sorted_idxs[sort_i], cur_max_height);
        seg_i = segments.length;
        candidates.push([sorted_idxs[sort_i], uv.at(sorted_idxs[sort_i])]);
        cur_max_height = 0;
      }
    }

    let c = candidates.shift();
    if (visited.at(c[0]) !== undefined || c[1] < cur_max_height) {
      continue;
    }
    cur_max_height = Math.max(cur_max_height, uv.at(c[0]));
    total_max_height = Math.max(cur_max_height, total_max_height);
    visited.set(c[0], seg_i);
    for (let u of [-1,1]) {
      for (let v of [-1,1]) {
        cuv = [u,v];
        candidates.push([cuv, uv.at(cuv)]);
      }
    }
  }
  return segments; 
}
