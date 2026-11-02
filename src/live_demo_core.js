/* live_demo_core.js -- the pipeline's algorithms (page: live_demo_page.html, built by make_live_demo.py), re-written small enough to run live in a browser.
   Mirrors src/p3_render.py (raycast scan), src/registration.py (plan method), src/p4_match_bim.py,
   src/p4_visibility.py and src/p7_score.py. Everything is seeded. Frame: BIM frame, z up, metres.
   Rule kept from the repo: registration, matching and visibility only see the cloud (xyz, rgb) and
   the survey-frame cameras. Ground truth lives in the EVAL object and is only used for scoring. */
(function (root) {
'use strict';
const Core = {};

// ---------- small helpers ----------
function rng(seed) {
  let a = seed >>> 0;
  const r = () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  r.normal = () => { let u = 0; while (u === 0) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
  return r;
}
function hash3(a, b, c) { let h = (Math.imul(a | 0, 73856093) ^ Math.imul(b | 0, 19349663) ^ Math.imul(c | 0, 83492791)) >>> 0; h = Math.imul(h ^ (h >>> 13), 0x5bd1e995) >>> 0; return ((h ^ (h >>> 15)) >>> 0) / 4294967296; }
function vnoise(x, y) {   // smooth value noise, ~[-1, 1]
  const xi = Math.floor(x), yi = Math.floor(y), fx = x - xi, fy = y - yi;
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
  const a = hash3(xi, yi, 7), b = hash3(xi + 1, yi, 7), c = hash3(xi, yi + 1, 7), d = hash3(xi + 1, yi + 1, 7);
  return 2 * ((a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy) - 1;
}
Core.rng = rng; Core.hash3 = hash3; Core.vnoise = vnoise;

// 4x4 row-major transforms
const M = {
  eye() { const m = new Float64Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; },
  mul(a, b) { const m = new Float64Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) { let s = 0; for (let k = 0; k < 4; k++) s += a[4 * i + k] * b[4 * k + j]; m[4 * i + j] = s; } return m; },
  trans(x, y, z) { const m = M.eye(); m[3] = x; m[7] = y; m[11] = z; return m; },
  rotAxis(ax, ay, az, deg) {
    const n = Math.hypot(ax, ay, az) || 1; ax /= n; ay /= n; az /= n;
    const t = deg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t), C = 1 - c, m = M.eye();
    m[0] = c + ax * ax * C; m[1] = ax * ay * C - az * s; m[2] = ax * az * C + ay * s;
    m[4] = ay * ax * C + az * s; m[5] = c + ay * ay * C; m[6] = ay * az * C - ax * s;
    m[8] = az * ax * C - ay * s; m[9] = az * ay * C + ax * s; m[10] = c + az * az * C; return m;
  },
  rotZ(deg) { return M.rotAxis(0, 0, 1, deg); },
  about(R, cx, cy, cz) { return M.mul(M.trans(cx, cy, cz), M.mul(R, M.trans(-cx, -cy, -cz))); },
  inv(m) {   // rigid (rotation + translation) inverse
    const r = M.eye();
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r[4 * i + j] = m[4 * j + i];
    for (let i = 0; i < 3; i++) r[4 * i + 3] = -(r[4 * i] * m[3] + r[4 * i + 1] * m[7] + r[4 * i + 2] * m[11]);
    return r;
  },
  apply(m, src, dst) {   // Float32Array xyz -> xyz
    const n = src.length / 3; dst = dst || new Float32Array(src.length);
    for (let i = 0; i < n; i++) {
      const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2];
      dst[3 * i] = m[0] * x + m[1] * y + m[2] * z + m[3];
      dst[3 * i + 1] = m[4] * x + m[5] * y + m[6] * z + m[7];
      dst[3 * i + 2] = m[8] * x + m[9] * y + m[10] * z + m[11];
    }
    return dst;
  },
  applyDir(m, src, dst) {
    const n = src.length / 3; dst = dst || new Float32Array(src.length);
    for (let i = 0; i < n; i++) {
      const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2];
      dst[3 * i] = m[0] * x + m[1] * y + m[2] * z; dst[3 * i + 1] = m[4] * x + m[5] * y + m[6] * z; dst[3 * i + 2] = m[8] * x + m[9] * y + m[10] * z;
    }
    return dst;
  },
  yawDeg(m) { return Math.atan2(m[4], m[0]) * 180 / Math.PI; },
  lerp(a, b, t) {   // pose interpolation for animation: slerp of yaw + tilt via matrix blend, re-orthonormalised
    const m = new Float64Array(16);
    for (let i = 0; i < 16; i++) m[i] = a[i] + (b[i] - a[i]) * t;
    // Gram-Schmidt on rotation rows' columns
    const c0 = [m[0], m[4], m[8]], c1 = [m[1], m[5], m[9]];
    let n = Math.hypot(...c0); c0[0] /= n; c0[1] /= n; c0[2] /= n;
    const d = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2]; c1[0] -= d * c0[0]; c1[1] -= d * c0[1]; c1[2] -= d * c0[2];
    n = Math.hypot(...c1); c1[0] /= n; c1[1] /= n; c1[2] /= n;
    const c2 = [c0[1] * c1[2] - c0[2] * c1[1], c0[2] * c1[0] - c0[0] * c1[2], c0[0] * c1[1] - c0[1] * c1[0]];
    m[0] = c0[0]; m[4] = c0[1]; m[8] = c0[2]; m[1] = c1[0]; m[5] = c1[1]; m[9] = c1[2]; m[2] = c2[0]; m[6] = c2[1]; m[10] = c2[2];
    m[12] = m[13] = m[14] = 0; m[15] = 1; return m;
  },
};
Core.M = M;

// ---------- data ----------
function b64bytes(s) {
  if (typeof Buffer !== 'undefined' && typeof atob === 'undefined') { const b = Buffer.from(s, 'base64'); return new Uint8Array(b).slice(); }
  const bin = atob(s), u = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
  return u;
}

Core.loadBim = function (D) {
  const vq = new Uint16Array(b64bytes(D.verts).buffer), f = new Uint16Array(b64bytes(D.faces).buffer);
  const triElem = new Uint16Array(b64bytes(D.tri_elem).buffer), state = b64bytes(D.state);
  const nT = D.n_faces, nE = D.elem.gid.length, o = D.origin;
  const tri = new Float32Array(nT * 9);
  for (let t = 0; t < nT; t++) for (let c = 0; c < 3; c++) { const v = f[3 * t + c]; for (let k = 0; k < 3; k++) tri[9 * t + 3 * c + k] = vq[3 * v + k] / 1000 + o[k]; }
  let lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
  for (let i = 0; i < tri.length; i += 3) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], tri[i + k]); hi[k] = Math.max(hi[k], tri[i + k]); }
  const area = new Float32Array(nE);
  for (let t = 0; t < nT; t++) area[triElem[t]] += triArea(tri, t);
  const bim = {
    tri, triElem, nT, nE, elem: D.elem, cats: D.cats, types: D.types, storeys: D.storeys, stages: D.stages,
    nStages: D.stages.length, stateAt: (s, e) => state[s * nE + e], lo, hi, area,
    center: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2],
    radius: Math.hypot(hi[0] - lo[0], hi[1] - lo[1]) / 2, mainTable: D.main_table, recorded: D.recorded || {},
  };
  if (D.busy) bim.busy = loadBusy(D.busy);
  if (bim.recorded.status) { const st = b64bytes(bim.recorded.status); bim.recordedStatus = s => st.subarray(s * nE, (s + 1) * nE); }
  return bim;
};
// busy site (p3_render.py --site busy): clutter items as triangle soups, as-built offsets, late / misplaced per stage
function loadBusy(B) {
  const o = B.origin, items = B.items.map(it => {
    let tri;
    if (it.poles) {   // vertical square bars, rebuilt from (x, y, z0, z1)
      const p = new Uint16Array(b64bytes(it.poles).buffer), n = p.length / 4, h = it.t / 2;
      tri = new Float32Array(n * 12 * 9);
      const F = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]];
      for (let i = 0; i < n; i++) {
        const x = p[4 * i] / 1000 + o[0], y = p[4 * i + 1] / 1000 + o[1], z0 = p[4 * i + 2] / 1000 + o[2], z1 = p[4 * i + 3] / 1000 + o[2], V = [];
        for (const a of [-h, h]) for (const b of [-h, h]) for (const z of [z0, z1]) V.push([x + a, y + b, z]);
        F.forEach((f, k) => { for (let c = 0; c < 3; c++) tri.set(V[f[c]], (i * 12 + k) * 9 + 3 * c); });
      }
    } else {
      const v = new Uint16Array(b64bytes(it.v).buffer), fb = b64bytes(it.f), f = it.w === 4 ? new Uint32Array(fb.buffer) : new Uint16Array(fb.buffer);
      tri = new Float32Array(f.length * 3);
      for (let i = 0; i < f.length; i++) for (let k = 0; k < 3; k++) tri[3 * i + k] = v[3 * f[i] + k] / 1000 + o[k];
    }
    let lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (let i = 0; i < tri.length; i += 3) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], tri[i + k]); hi[k] = Math.max(hi[k], tri[i + k]); }
    return { cls: it.c, tri, lo, hi };
  });
  const offset = new Map(Object.entries(B.offset).map(([k, v]) => [+k, v]));
  return { classes: B.classes, rgb: B.rgb, items, offset, deviated: new Set(B.deviated),
    stages: B.stages.map(s => ({ items: s.items, missing: new Set(s.missing), misplaced: new Set(s.misplaced) })) };
}
const SMALL_CLUTTER = new Set(['pile', 'worker', 'machinery', 'mach_red', 'mach_blue', 'mach_dark', 'mach_metal', 'temp_wood', 'temp_blue', 'temp_red', 'temp_orange', 'temp_metal', 'temp_dark']);
function triArea(tri, t) {
  const o = 9 * t, ax = tri[o + 3] - tri[o], ay = tri[o + 4] - tri[o + 1], az = tri[o + 5] - tri[o + 2];
  const bx = tri[o + 6] - tri[o], by = tri[o + 7] - tri[o + 1], bz = tri[o + 8] - tri[o + 2];
  return 0.5 * Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
}

// Triangles of one construction stage. Partial elements: every vertex above the cut height is
// clamped down to it (src/p2_p3_simulate_stages.py clip_partial_height), giving the shorter prism.
// Busy site (opts.busy): late elements are left out, off-design / misplaced ones are moved, and the
// stage's clutter is added with triElem = nE + clutter class. Clutter that would swallow an indoor
// scanner position is dropped (site_clutter.drop_blocking); pass those eyes as opts.eyes.
Core.stageTris = function (bim, s, opts) {
  opts = opts || {};
  const B = opts.busy && bim.busy, bs = B && B.stages[s], out = [], el = [];
  for (let t = 0; t < bim.nT; t++) {
    const e = bim.triElem[t], v = bim.stateAt(s, e);
    if (v === 0 || (bs && bs.missing.has(e))) continue;
    let cut = Infinity;
    if (v < 255) cut = bim.elem.zmin[e] + (v / 254) * (bim.elem.zmax[e] - bim.elem.zmin[e]);
    const o = 9 * t, a = new Array(9);
    for (let i = 0; i < 9; i++) a[i] = bim.tri[o + i];
    if (cut < Infinity) { a[2] = Math.min(a[2], cut); a[5] = Math.min(a[5], cut); a[8] = Math.min(a[8], cut); }
    const d = B && B.offset.get(e);
    if (d) for (let k = 0; k < 3; k++) { a[3 * k] += d[0]; a[3 * k + 1] += d[1]; }
    const tmp = new Float32Array(a);
    if (triArea(tmp, 0) < 1e-7) continue;
    for (let i = 0; i < 9; i++) out.push(a[i]);
    el.push(e);
  }
  let n = el.length, clutter = [];
  if (bs) clutter = bs.items.map(i => B.items[i]).filter(it => !(SMALL_CLUTTER.has(B.classes[it.cls]) && (opts.eyes || []).some(p =>
    p[0] > it.lo[0] - 0.5 && p[0] < it.hi[0] + 0.5 && p[1] > it.lo[1] - 0.5 && p[1] < it.hi[1] + 0.5 && p[2] > it.lo[2] - 0.5 && p[2] < it.hi[2] + 0.5)));
  const nc = clutter.reduce((a, it) => a + it.tri.length / 9, 0);
  const tri = new Float32Array((n + nc) * 9), triElem = new Uint16Array(n + nc);
  tri.set(out); triElem.set(el);
  for (const it of clutter) { tri.set(it.tri, 9 * n); triElem.fill(bim.nE + it.cls, n, n + it.tri.length / 9); n += it.tri.length / 9; }
  return { tri, triElem, n, nDesign: el.length, clutter };
};
// Full design triangles for a subset of elements (keep(e) -> bool)
Core.subsetTris = function (bim, keep) {
  let n = 0;
  for (let t = 0; t < bim.nT; t++) if (keep(bim.triElem[t])) n++;
  const tri = new Float32Array(n * 9), triElem = new Uint16Array(n);
  let j = 0;
  for (let t = 0; t < bim.nT; t++) {
    const e = bim.triElem[t];
    if (!keep(e)) continue;
    tri.set(bim.tri.subarray(9 * t, 9 * t + 9), 9 * j); triElem[j++] = e;
  }
  return { tri, triElem, n };
};

// ---------- BVH: first hit, all hits, closest point ----------
function BVH(soup) {
  const n = soup.n, src = soup.tri;
  const cen = new Float32Array(n * 3), bmin = new Float32Array(n * 3), bmax = new Float32Array(n * 3);
  for (let t = 0; t < n; t++) for (let k = 0; k < 3; k++) {
    const a = src[9 * t + k], b = src[9 * t + 3 + k], c = src[9 * t + 6 + k];
    bmin[3 * t + k] = Math.min(a, b, c); bmax[3 * t + k] = Math.max(a, b, c); cen[3 * t + k] = (a + b + c) / 3;
  }
  const idx = new Uint32Array(n); for (let i = 0; i < n; i++) idx[i] = i;
  const maxN = Math.max(1, 2 * n);
  const nb = new Float32Array(maxN * 6), nl = new Int32Array(maxN * 2);
  let nNodes = 1;
  const stack = [[0, 0, n]];
  while (stack.length) {
    const [node, s, e] = stack.pop();
    let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity, hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
    let c0 = Infinity, c1 = Infinity, c2 = Infinity, d0 = -Infinity, d1 = -Infinity, d2 = -Infinity;
    for (let i = s; i < e; i++) {
      const t = idx[i];
      lo0 = Math.min(lo0, bmin[3 * t]); lo1 = Math.min(lo1, bmin[3 * t + 1]); lo2 = Math.min(lo2, bmin[3 * t + 2]);
      hi0 = Math.max(hi0, bmax[3 * t]); hi1 = Math.max(hi1, bmax[3 * t + 1]); hi2 = Math.max(hi2, bmax[3 * t + 2]);
      c0 = Math.min(c0, cen[3 * t]); c1 = Math.min(c1, cen[3 * t + 1]); c2 = Math.min(c2, cen[3 * t + 2]);
      d0 = Math.max(d0, cen[3 * t]); d1 = Math.max(d1, cen[3 * t + 1]); d2 = Math.max(d2, cen[3 * t + 2]);
    }
    nb.set([lo0, lo1, lo2, hi0, hi1, hi2], 6 * node);
    const cnt = e - s;
    if (cnt <= 4) { nl[2 * node] = s; nl[2 * node + 1] = -cnt; continue; }
    const ext = [d0 - c0, d1 - c1, d2 - c2], ax = ext[0] > ext[1] ? (ext[0] > ext[2] ? 0 : 2) : (ext[1] > ext[2] ? 1 : 2);
    const mid = ax === 0 ? (c0 + d0) / 2 : ax === 1 ? (c1 + d1) / 2 : (c2 + d2) / 2;
    let i = s, j = e - 1;
    while (i <= j) { if (cen[3 * idx[i] + ax] < mid) i++; else { const tmp = idx[i]; idx[i] = idx[j]; idx[j] = tmp; j--; } }
    let m = i;
    if (m === s || m === e) {   // all centroids equal along the axis: split by count
      const sub = Array.from(idx.subarray(s, e)).sort((p, q) => cen[3 * p + ax] - cen[3 * q + ax]);
      idx.set(sub, s); m = (s + e) >> 1;
    }
    const L = nNodes++, R = nNodes++;
    nl[2 * node] = L; nl[2 * node + 1] = R;
    stack.push([L, s, m], [R, m, e]);
  }
  // re-ordered triangle data + per-triangle normals
  this.t = new Float32Array(n * 9); this.e = new Uint16Array(n); this.nrm = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const t = idx[i];
    this.t.set(src.subarray(9 * t, 9 * t + 9), 9 * i); this.e[i] = soup.triElem[t];
    const o = 9 * i, T = this.t;
    const ax = T[o + 3] - T[o], ay = T[o + 4] - T[o + 1], az = T[o + 5] - T[o + 2], bx = T[o + 6] - T[o], by = T[o + 7] - T[o + 1], bz = T[o + 8] - T[o + 2];
    let nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx; const l = Math.hypot(nx, ny, nz) || 1;
    this.nrm[3 * i] = nx / l; this.nrm[3 * i + 1] = ny / l; this.nrm[3 * i + 2] = nz / l;
  }
  this.nb = nb; this.nl = nl; this.n = n; this.stk = new Int32Array(1024);
  this.hitTri = -1; this.cd = Infinity; this.ct = -1; this.cp = new Float64Array(3);
}
// first hit along o + t d, t in (eps, tmax); returns t or Infinity, sets hitTri
BVH.prototype.intersect = function (ox, oy, oz, dx, dy, dz, tmax, anyHit) {
  if (this.n === 0) return Infinity;
  const nb = this.nb, nl = this.nl, T = this.t, st = this.stk;
  const ix = 1 / dx, iy = 1 / dy, iz = 1 / dz;
  let best = tmax, hit = -1, sp = 0; st[sp++] = 0;
  while (sp) {
    const nd = st[--sp], b = 6 * nd;
    let t1 = (nb[b] - ox) * ix, t2 = (nb[b + 3] - ox) * ix;
    let tn = Math.min(t1, t2), tf = Math.max(t1, t2);
    t1 = (nb[b + 1] - oy) * iy; t2 = (nb[b + 4] - oy) * iy; tn = Math.max(tn, Math.min(t1, t2)); tf = Math.min(tf, Math.max(t1, t2));
    t1 = (nb[b + 2] - oz) * iz; t2 = (nb[b + 5] - oz) * iz; tn = Math.max(tn, Math.min(t1, t2)); tf = Math.min(tf, Math.max(t1, t2));
    if (tf < Math.max(tn, 0) || tn > best) continue;
    const a = nl[2 * nd], c = nl[2 * nd + 1];
    if (c > 0 || (c === 0 && a > 0)) { st[sp++] = a; st[sp++] = c; continue; }
    for (let k = a, end = a - c; k < end; k++) {
      const o = 9 * k;
      const e1x = T[o + 3] - T[o], e1y = T[o + 4] - T[o + 1], e1z = T[o + 5] - T[o + 2];
      const e2x = T[o + 6] - T[o], e2y = T[o + 7] - T[o + 1], e2z = T[o + 8] - T[o + 2];
      const px = dy * e2z - dz * e2y, py = dz * e2x - dx * e2z, pz = dx * e2y - dy * e2x;
      const det = e1x * px + e1y * py + e1z * pz;
      if (det > -1e-12 && det < 1e-12) continue;
      const inv = 1 / det, sx = ox - T[o], sy = oy - T[o + 1], sz = oz - T[o + 2];
      const u = (sx * px + sy * py + sz * pz) * inv; if (u < 0 || u > 1) continue;
      const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
      const v = (dx * qx + dy * qy + dz * qz) * inv; if (v < 0 || u + v > 1) continue;
      const t = (e2x * qx + e2y * qy + e2z * qz) * inv;
      if (t > 1e-4 && t < best) { best = t; hit = k; if (anyHit) { this.hitTri = k; return t; } }
    }
  }
  this.hitTri = hit; return hit < 0 ? Infinity : best;
};
// every hit with t < tmax: calls cb(t, tri)
BVH.prototype.allHits = function (ox, oy, oz, dx, dy, dz, tmax, cb) {
  if (this.n === 0) return;
  const nb = this.nb, nl = this.nl, T = this.t, st = this.stk;
  const ix = 1 / dx, iy = 1 / dy, iz = 1 / dz;
  let sp = 0; st[sp++] = 0;
  while (sp) {
    const nd = st[--sp], b = 6 * nd;
    let t1 = (nb[b] - ox) * ix, t2 = (nb[b + 3] - ox) * ix;
    let tn = Math.min(t1, t2), tf = Math.max(t1, t2);
    t1 = (nb[b + 1] - oy) * iy; t2 = (nb[b + 4] - oy) * iy; tn = Math.max(tn, Math.min(t1, t2)); tf = Math.min(tf, Math.max(t1, t2));
    t1 = (nb[b + 2] - oz) * iz; t2 = (nb[b + 5] - oz) * iz; tn = Math.max(tn, Math.min(t1, t2)); tf = Math.min(tf, Math.max(t1, t2));
    if (tf < Math.max(tn, 0) || tn > tmax) continue;
    const a = nl[2 * nd], c = nl[2 * nd + 1];
    if (c > 0 || (c === 0 && a > 0)) { st[sp++] = a; st[sp++] = c; continue; }
    for (let k = a, end = a - c; k < end; k++) {
      const o = 9 * k;
      const e1x = T[o + 3] - T[o], e1y = T[o + 4] - T[o + 1], e1z = T[o + 5] - T[o + 2];
      const e2x = T[o + 6] - T[o], e2y = T[o + 7] - T[o + 1], e2z = T[o + 8] - T[o + 2];
      const px = dy * e2z - dz * e2y, py = dz * e2x - dx * e2z, pz = dx * e2y - dy * e2x;
      const det = e1x * px + e1y * py + e1z * pz;
      if (det > -1e-12 && det < 1e-12) continue;
      const inv = 1 / det, sx = ox - T[o], sy = oy - T[o + 1], sz = oz - T[o + 2];
      const u = (sx * px + sy * py + sz * pz) * inv; if (u < 0 || u > 1) continue;
      const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
      const v = (dx * qx + dy * qy + dz * qz) * inv; if (v < 0 || u + v > 1) continue;
      const t = (e2x * qx + e2y * qy + e2z * qz) * inv;
      if (t > 1e-4 && t < tmax) cb(t, k);
    }
  }
};
// closest surface point within maxDist, skipping triangles of element `skip` (-1: none).
// Returns the distance (Infinity if none); sets ct (triangle) and cp (point).
BVH.prototype.closest = function (px, py, pz, maxDist, skip) {
  if (this.n === 0) return Infinity;
  const nb = this.nb, nl = this.nl, T = this.t, st = this.stk, E = this.e;
  let best2 = maxDist * maxDist, bt = -1, bx = 0, by = 0, bz = 0, sp = 0; st[sp++] = 0;
  while (sp) {
    const nd = st[--sp], b = 6 * nd;
    const dx = Math.max(nb[b] - px, 0, px - nb[b + 3]), dy = Math.max(nb[b + 1] - py, 0, py - nb[b + 4]), dz = Math.max(nb[b + 2] - pz, 0, pz - nb[b + 5]);
    if (dx * dx + dy * dy + dz * dz > best2) continue;
    const a = nl[2 * nd], c = nl[2 * nd + 1];
    if (c > 0 || (c === 0 && a > 0)) { st[sp++] = a; st[sp++] = c; continue; }
    for (let k = a, end = a - c; k < end; k++) {
      if (E[k] === skip) continue;
      const q = closestPtTri(px, py, pz, T, 9 * k);
      const ex = q[0] - px, ey = q[1] - py, ez = q[2] - pz, d2 = ex * ex + ey * ey + ez * ez;
      if (d2 < best2) { best2 = d2; bt = k; bx = q[0]; by = q[1]; bz = q[2]; }
    }
  }
  this.ct = bt; this.cp[0] = bx; this.cp[1] = by; this.cp[2] = bz;
  return bt < 0 ? Infinity : Math.sqrt(best2);
};
const _q = new Float64Array(3);
function closestPtTri(px, py, pz, T, o) {   // Ericson, Real-Time Collision Detection 5.1.5
  const ax = T[o], ay = T[o + 1], az = T[o + 2], bx = T[o + 3], by = T[o + 4], bz = T[o + 5], cx = T[o + 6], cy = T[o + 7], cz = T[o + 8];
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = px - ax, apy = py - ay, apz = pz - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0 && d2 <= 0) { _q[0] = ax; _q[1] = ay; _q[2] = az; return _q; }
  const bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) { _q[0] = bx; _q[1] = by; _q[2] = bz; return _q; }
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); _q[0] = ax + v * abx; _q[1] = ay + v * aby; _q[2] = az + v * abz; return _q; }
  const cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) { _q[0] = cx; _q[1] = cy; _q[2] = cz; return _q; }
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); _q[0] = ax + w * acx; _q[1] = ay + w * acy; _q[2] = az + w * acz; return _q; }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) { const w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); _q[0] = bx + w * (cx - bx); _q[1] = by + w * (cy - by); _q[2] = bz + w * (cz - bz); return _q; }
  const den = 1 / (va + vb + vc), v = vb * den, w = vc * den;
  _q[0] = ax + abx * v + acx * w; _q[1] = ay + aby * v + acy * w; _q[2] = az + abz * v + acz * w; return _q;
}
Core.BVH = BVH;

// ---------- materials: what the camera sees (p3_render.py RENDER_RGB, not the catalogue) ----------
Core.RENDER_RGB = {
  brick_brown: [128, 70, 50], brick_yellow: [184, 146, 94], calcium_silicate: [196, 196, 189],
  aerated_concrete: [206, 205, 199], gypsum_block: [214, 213, 207], concrete_prefab: [172, 171, 166],
  concrete_cast: [150, 149, 145], screed: [146, 140, 131], steel_dark: [54, 58, 65],
  steel_primer: [126, 58, 45], wood: [134, 92, 56], plywood: [184, 150, 104],
  board_white: [226, 226, 221], zinc: [138, 146, 151], tile: [108, 103, 97], insulation: [232, 220, 168],
};
Core.MORTAR = [176, 172, 164];
// (course height, unit length, joint width) m -- brick from p3_render.py COURSES; blocks are larger units
Core.COURSES = { brick_brown: [0.0625, 0.22, 0.012], brick_yellow: [0.0625, 0.22, 0.012],
  calcium_silicate: [0.25, 0.9, 0.006], aerated_concrete: [0.25, 0.6, 0.006], gypsum_block: [0.5, 0.666, 0.004] };
// surface pattern multiplier at texture coords (u along the surface, v up); returns <0 for a mortar joint
Core.pattern = function (cat, u, v) {
  const C = Core.COURSES[cat];
  if (C) {
    const [ch, ul, j] = C, row = Math.floor(v / ch), off = (row & 1) * ul / 2;
    const fu = ((u + off) % ul + ul) % ul, fv = ((v % ch) + ch) % ch;
    if (fu < j || fv < j) return -1;
    return 1 + 0.16 * (hash3(row, Math.floor((u + off) / ul), 3) - 0.5);
  }
  if (cat.startsWith('concrete') || cat === 'screed') return 1 + 0.07 * vnoise(u * 1.7, v * 1.7) + 0.03 * vnoise(u * 9, v * 9);
  if (cat === 'wood' || cat === 'plywood') return 1 + 0.09 * Math.sin(v * 38 + 3 * vnoise(u * 2, v * 0.5));
  if (cat === 'zinc') return 1 + 0.06 * ((((u % 0.5) + 0.5) % 0.5) < 0.03 ? 1 : 0);
  return 1 + 0.025 * vnoise(u * 4, v * 4);
};
// texture coordinates of a surface point: u runs along a wall, v is height; floors use x, y
Core.texUV = function (x, y, z, nx, ny, nz) {
  if (Math.abs(nz) > 0.7) return [x, y];
  const l = Math.hypot(nx, ny) || 1;
  return [x * (-ny / l) + y * (nx / l), z];
};

// ---------- the scan (p3_render.py): drone orbit + indoor walk-through, first hits only ----------
Core.SCAN = { N_ORBIT: 20, N_BANDS: 3, EXT_FOV: 70, EXT_GRID: [60, 40], INT_GRID: [3, 3], INT_RAYS: 2000,
  NOISE: 0.015, DROPOUT: 0.15, SUN_AZ: 215, SUN_EL: 38 };

Core.viewpoints = function (bim, s) {
  const S = Core.SCAN, c = bim.center, r = bim.radius, views = [];
  const dist = r * 1.7 + 3, h0 = bim.lo[2], h1 = bim.hi[2];
  for (let band = 0; band < S.N_BANDS; band++) {
    const z = h0 + (0.35 + 0.5 * band / (S.N_BANDS - 1)) * (h1 - h0) + 2;
    for (let i = 0; i < S.N_ORBIT; i++) {
      const th = 2 * Math.PI * (i + band * 0.33) / S.N_ORBIT;
      views.push({ kind: 'drone', eye: [c[0] + dist * Math.cos(th), c[1] + dist * Math.sin(th), z], target: [c[0], c[1], (h0 + h1) / 2] });
    }
  }
  // one indoor station grid per built floor level (slab tops merged within 1.5 m)
  const tops = [];
  for (let e = 0; e < bim.nE; e++) if (bim.elem.type[e] === 2 && bim.stateAt(s, e) === 255) tops.push(Math.round(bim.elem.zmax[e] * 10) / 10);
  tops.sort((a, b) => a - b);
  const levels = [];
  for (const z of tops) if (!levels.length || z - levels[levels.length - 1] > 1.5) levels.push(z);
  const inset = r * 0.45;
  for (const z of levels) for (let i = 0; i < S.INT_GRID[0]; i++) for (let j = 0; j < S.INT_GRID[1]; j++) {
    const fx = (i + 0.5) / S.INT_GRID[0] * 2 - 1, fy = (j + 0.5) / S.INT_GRID[1] * 2 - 1;
    views.push({ kind: 'indoor', eye: [c[0] + fx * inset, c[1] + fy * inset, z + 1.4], level: z });
  }
  return views;
};

Core.viewRays = function (view, seed) {
  const S = Core.SCAN, dirs = [];
  if (view.kind === 'drone') {
    const [w, h] = S.EXT_GRID, e = view.eye, t = view.target;
    let fx = t[0] - e[0], fy = t[1] - e[1], fz = t[2] - e[2]; let l = Math.hypot(fx, fy, fz); fx /= l; fy /= l; fz /= l;
    let rx = fy, ry = -fx, rz = 0; l = Math.hypot(rx, ry); rx /= l; ry /= l;
    const ux = ry * fz - rz * fy, uy = rz * fx - rx * fz, uz = rx * fy - ry * fx;
    const tw = Math.tan(S.EXT_FOV * Math.PI / 360), th = tw / (w / h);
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
      const a = tw * (2 * i / (w - 1) - 1), b = th * (2 * j / (h - 1) - 1);
      let dx = fx + a * rx + b * ux, dy = fy + a * ry + b * uy, dz = fz + a * rz + b * uz; l = Math.hypot(dx, dy, dz);
      dirs.push(dx / l, dy / l, dz / l);
    }
  } else {   // Fibonacci sphere with a small random spin (no ring aliasing)
    const n = S.INT_RAYS, R = rng(seed), rot = R() * 2 * Math.PI, ga = Math.PI * (3 - Math.sqrt(5));
    for (let i = 0; i < n; i++) {
      const z = 1 - 2 * (i + 0.5) / n, rr = Math.sqrt(1 - z * z), phi = i * ga + rot;
      dirs.push(rr * Math.cos(phi), rr * Math.sin(phi), z);
    }
  }
  return new Float32Array(dirs);
};

// clutter that is not in the BIM: ground disc and four trees (same every stage, as in the repo)
Core.siteClutter = function (bim) {
  const c = bim.center, r = bim.radius, R = rng(11), trees = [];
  for (let i = 0; i < 4; i++) {
    const a = 2 * Math.PI * (i / 4 + 0.12 + 0.1 * R()), d = r * (1.2 + 0.25 * R());
    trees.push({ x: c[0] + d * Math.cos(a), y: c[1] + d * Math.sin(a), z: bim.lo[2] + 3.4 + R(), rad: 2.1 + 0.9 * R() });
  }
  return { groundZ: bim.lo[2] - 0.05, groundR: r * 2.3, trees };
};

// Scanner: call step(k) to process the next k views. Points are made in the BIM frame and only
// moved into the survey frame at the end (finish), like a real device that does not know where it is.
// opts.site 'clean' | 'busy' (scaffold, crane, props, piles, workers, machinery, as-built errors)
// opts.sensor 'simple' (1.5 cm noise, 15% dropout) | 'realistic' (p3_render.py realistic_sensor)
Core.RS = { SIGMA0: 0.005, SIGMA_PER_M: 0.0008, MIN_COS: 0.2, BASE_DROP: 0.10, GRAZING_COS: 0.3,
  HOLE_AREA: 60, HOLE_R: [0.3, 1.5], MIXED: 0.01, FLOATER: 0.003 };
Core.Scanner = function (bim, s, seed, opts) {
  opts = opts || {};
  this.bim = bim; this.s = s; this.views = Core.viewpoints(bim, s);
  this.busy = opts.site === 'busy' && !!bim.busy; this.realistic = opts.sensor === 'realistic';
  this.soup = Core.stageTris(bim, s, { busy: this.busy, eyes: this.views.filter(v => v.kind === 'indoor').map(v => v.eye) });
  this.bvh = new BVH(this.soup);
  if (this.realistic) {   // texture-less holes in the drone (photo) points, spread over the built surface
    const H = rng(seed + 303), cum = [];
    let A = 0;
    for (let t = 0; t < this.soup.nDesign; t++) { A += triArea(this.soup.tri, t); cum.push(A); }
    const nH = Math.max(1, Math.round(A * 0.5 / Core.RS.HOLE_AREA));
    this.holes = [];
    for (let i = 0; i < nH; i++) {
      let lo = 0, hi = cum.length - 1; const x = H() * A;
      while (lo < hi) { const m = (lo + hi) >> 1; if (cum[m] < x) lo = m + 1; else hi = m; }
      const o = 9 * lo, T = this.soup.tri; let u = H(), v = H(); if (u + v > 1) { u = 1 - u; v = 1 - v; }
      const r = Core.RS.HOLE_R[0] + H() * (Core.RS.HOLE_R[1] - Core.RS.HOLE_R[0]);
      this.holes.push([T[o] + u * (T[o + 3] - T[o]) + v * (T[o + 6] - T[o]), T[o + 1] + u * (T[o + 4] - T[o + 1]) + v * (T[o + 7] - T[o + 1]), T[o + 2] + u * (T[o + 5] - T[o + 2]) + v * (T[o + 8] - T[o + 2]), r * r]);
    }
  }
  this.site = Core.siteClutter(bim); this.R = rng(seed); this.CR = rng(seed + 101);
  this.next = 0; this.pts = []; this.rgb = []; this.gtElem = []; this.rayO = []; this.rayD = []; this.rayHit = [];
  const S = Core.SCAN, az = S.SUN_AZ * Math.PI / 180, el = S.SUN_EL * Math.PI / 180;
  this.sun = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
  this.elemJit = new Float32Array(bim.nE * 3);   // as-built deviation: a few cm per element
  const J = rng(seed + 202);
  for (let i = 0; i < bim.nE * 3; i++) this.elemJit[i] = J.normal() * 0.012;
  this.tint = new Float32Array(bim.nE);
  for (let e = 0; e < bim.nE; e++) this.tint[e] = 1 + 0.08 * (J() - 0.5);
};
Core.Scanner.prototype.done = function () { return this.next >= this.views.length; };
Core.Scanner.prototype.step = function (k) {
  const out = { view: null, hits: [], nRays: 0 };
  for (let q = 0; q < k && !this.done(); q++) {
    const vi = this.next++, v = this.views[vi]; out.view = v;
    const dirs = Core.viewRays(v, 1000 + vi), n = dirs.length / 3, R = this.R, CR = this.CR, S = Core.SCAN;
    const exposure = 1 + 0.05 * CR.normal(), tmax = v.kind === 'drone' ? 80 : 25;
    out.nRays += n;
    for (let i = 0; i < n; i++) {
      const dx = dirs[3 * i], dy = dirs[3 * i + 1], dz = dirs[3 * i + 2], [ox, oy, oz] = v.eye;
      let t = this.bvh.intersect(ox, oy, oz, dx, dy, dz, tmax), kind = t < Infinity ? 'bim' : 'none', tri = this.bvh.hitTri;
      let nx = 0, ny = 0, nz = 1;
      if (dz < 0) { const tg = (this.site.groundZ - oz) / dz; const gx = ox + tg * dx - this.bim.center[0], gy = oy + tg * dy - this.bim.center[1];
        if (tg < t && gx * gx + gy * gy < this.site.groundR ** 2) { t = tg; kind = 'ground'; } }
      for (const tr of this.site.trees) {   // ray-sphere
        const lx = ox - tr.x, ly = oy - tr.y, lz = oz - tr.z, b = lx * dx + ly * dy + lz * dz, cc = lx * lx + ly * ly + lz * lz - tr.rad * tr.rad, disc = b * b - cc;
        if (disc > 0) { const tt = -b - Math.sqrt(disc); if (tt > 0 && tt < t) { t = tt; kind = 'tree'; nx = (lx + tt * dx) / tr.rad; ny = (ly + tt * dy) / tr.rad; nz = (lz + tt * dz) / tr.rad; } }
      }
      this.rayO.push(vi); this.rayD.push(dx, dy, dz);
      if (kind === 'none') { this.rayHit.push(0); continue; }
      this.rayHit.push(1);
      let px = ox + t * dx, py = oy + t * dy, pz = oz + t * dz, e = -1;
      if (kind === 'bim') {
        e = this.bvh.e[tri]; nx = this.bvh.nrm[3 * tri]; ny = this.bvh.nrm[3 * tri + 1]; nz = this.bvh.nrm[3 * tri + 2];
        if (nx * dx + ny * dy + nz * dz > 0) { nx = -nx; ny = -ny; nz = -nz; }
        if (e >= this.bim.nE) kind = 'clutter';
      }
      if (this.realistic) {   // dropout grows at grazing angles; photo points also lose texture-less patches
        const RS = Core.RS, cos = Math.abs(nx * dx + ny * dy + nz * dz);
        if (R() < RS.BASE_DROP + (1 - RS.BASE_DROP) * Math.min(1, Math.max(0, (RS.GRAZING_COS - cos) / RS.GRAZING_COS))) continue;
        if (v.kind === 'drone' && this.holes.some(h => (px - h[0]) ** 2 + (py - h[1]) ** 2 + (pz - h[2]) ** 2 < h[3])) continue;
      } else if (R() < S.DROPOUT) continue;   // sensor dropout
      const col = this.colour(kind, e, px, py, pz, nx, ny, nz, dx, dy, dz, t, v, exposure);
      if (kind === 'bim') { px += this.elemJit[3 * e]; py += this.elemJit[3 * e + 1]; pz += this.elemJit[3 * e + 2]; }
      if (this.realistic) {   // noise mostly along the ray, growing with range and grazing angle; a few edge ghosts
        const RS = Core.RS, cos = Math.abs(nx * dx + ny * dy + nz * dz), sg = (RS.SIGMA0 + RS.SIGMA_PER_M * t) / Math.max(cos, RS.MIN_COS);
        let k = R.normal() * sg;
        if (R() < RS.MIXED) k += 0.05 + 0.95 * R();
        px += dx * k + R.normal() * RS.SIGMA0; py += dy * k + R.normal() * RS.SIGMA0; pz += dz * k + R.normal() * RS.SIGMA0;
      } else { px += R.normal() * S.NOISE; py += R.normal() * S.NOISE; pz += R.normal() * S.NOISE; }
      this.pts.push(px, py, pz); this.rgb.push(col[0], col[1], col[2]);
      this.gtElem.push(kind === 'bim' ? e : kind === 'ground' ? -2 : kind === 'tree' ? -3 : -10 - (e - this.bim.nE));
      out.hits.push(px, py, pz, col[0], col[1], col[2]);
    }
  }
  return out;
};
Core.Scanner.prototype.colour = function (kind, e, x, y, z, nx, ny, nz, dx, dy, dz, t, v, exposure) {
  const CR = this.CR;
  let base, pat = 1;
  if (kind === 'ground') { base = [96, 88, 74]; pat = 1 + 0.12 * vnoise(x * 0.8, y * 0.8) + 0.05 * vnoise(x * 6, y * 6); }
  else if (kind === 'tree') { base = [76, 110, 68]; pat = 1 + 0.25 * (CR() - 0.5); }
  else if (kind === 'clutter') { const B = this.bim.busy; base = B.rgb[B.classes[e - this.bim.nE]]; pat = 1 + 0.12 * (CR() - 0.5); }
  else {
    const cat = this.bim.cats[this.bim.elem.mat[e]], uv = Core.texUV(x, y, z, nx, ny, nz);
    pat = Core.pattern(cat, uv[0], uv[1]);
    base = pat < 0 ? Core.MORTAR : Core.RENDER_RGB[cat];
    if (pat < 0) pat = 1;
    pat *= this.tint[e];
    if (z - this.bim.elem.zmin[e] < 0.25 && Math.abs(nz) < 0.5) pat *= 0.88;   // dirt at the foot
  }
  let shade;
  if (v.kind === 'drone') {
    const s = this.sun, lam = Math.max(0, nx * s[0] + ny * s[1] + nz * s[2]);
    let lit = lam;
    if (lam > 0) {   // shadow ray towards the sun
      const o = 0.03, tt = this.bvh.intersect(x + nx * o, y + ny * o, z + nz * o, s[0], s[1], s[2], 200, true);
      if (tt < Infinity) lit = 0;
    }
    shade = 0.42 + 0.72 * lit + 0.1 * nz;
  } else shade = (0.5 + 0.45 * Math.abs(nx * dx + ny * dy + nz * dz)) / (1 + 0.004 * t * t);
  const g = shade * exposure * pat;
  return [clamp255(base[0] * g + CR.normal() * 3), clamp255(base[1] * g + CR.normal() * 3), clamp255(base[2] * g + CR.normal() * 3)];
};
function clamp255(v) { return v < 0 ? 0 : v > 255 ? 255 : Math.round(v); }

// Finish: the survey device reports points + its camera poses in its own frame (T_true: BIM -> survey).
// Returns { cloud (what the method may use), eval (ground truth, scoring only) }.
Core.Scanner.prototype.finish = function (T_true) {
  let pts = this.pts, rgbA = this.rgb, gt = this.gtElem;
  if (this.realistic && pts.length) {   // a few random floaters in the bounding box
    const F = rng(9090 + this.s), n = Math.round(pts.length / 3 * Core.RS.FLOATER), lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (let i = 0; i < pts.length; i += 3) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], pts[i + k]); hi[k] = Math.max(hi[k], pts[i + k]); }
    pts = pts.slice(); rgbA = rgbA.slice(); gt = gt.slice();
    for (let i = 0; i < n; i++) { for (let k = 0; k < 3; k++) pts.push(lo[k] + F() * (hi[k] - lo[k])); const g = 60 + Math.floor(F() * 140); rgbA.push(g, g, g); gt.push(-5); }
  }
  const xyzB = new Float32Array(pts);
  const cloud = { xyz: M.apply(T_true, xyzB), rgb: new Uint8Array(rgbA),
    cams: this.views.map(v => { const p = M.apply(T_true, new Float32Array(v.eye)); return { kind: v.kind, eye: [p[0], p[1], p[2]] }; }),
    rayView: new Uint16Array(this.rayO), rayDir: M.applyDir(T_true, new Float32Array(this.rayD)), rayHit: new Uint8Array(this.rayHit) };
  const bs = this.busy ? this.bim.busy.stages[this.s] : null;
  const ev = { T_true, xyzBim: xyzB, gtElem: new Int32Array(gt), stage: this.s,
    late: bs ? bs.missing : new Set(), misplaced: bs ? bs.misplaced : new Set() };
  return { cloud, eval: ev };
};

// survey offsets (p2_p3_simulate_stages.py survey_offset + REG_JITTER), rotation about the building centre
Core.surveyOffset = function (bim, regime, seed, yawOverride) {
  const R = rng(seed), c = bim.center;
  let t, yaw;
  if (regime === 'gnss') { t = [R.normal() * 2, R.normal() * 2, R.normal() * 1]; yaw = (R() * 2 - 1) * 15; }
  else if (regime === 'basin') { t = [R.normal() * 1.5, R.normal() * 1.5, R.normal() * 0.5]; yaw = 0; }
  else { t = [(R() * 2 - 1) * 30, (R() * 2 - 1) * 30, (R() * 2 - 1) * 5]; yaw = (R() * 2 - 1) * 180; }
  if (yawOverride !== undefined) yaw = yawOverride;
  const tilt = M.rotAxis(R.normal(), R.normal(), 0, R.normal() * 0.5);
  const Rm = M.mul(tilt, M.rotZ(yaw));
  return { T: M.mul(M.trans(t[0], t[1], t[2]), M.about(Rm, c[0], c[1], c[2])), yaw, t };
};

// eval only: mean displacement of the cloud's points between the estimated and the true alignment
Core.regError = function (T_est, ev, stride) {
  const Tinv = M.inv(ev.T_true), xs = ev.xyzBim; stride = stride || 20;
  let s = 0, n = 0;
  for (let i = 0; i < xs.length / 3; i += stride) {
    // survey point = T_true * bim point; estimate maps it back with T_est
    const x = xs[3 * i], y = xs[3 * i + 1], z = xs[3 * i + 2], T = ev.T_true;
    const sx = T[0] * x + T[1] * y + T[2] * z + T[3], sy = T[4] * x + T[5] * y + T[6] * z + T[7], sz = T[8] * x + T[9] * y + T[10] * z + T[11];
    const ex = T_est[0] * sx + T_est[1] * sy + T_est[2] * sz + T_est[3], ey = T_est[4] * sx + T_est[5] * sy + T_est[6] * sz + T_est[7], ez = T_est[8] * sx + T_est[9] * sy + T_est[10] * sz + T_est[11];
    s += Math.hypot(ex - x, ey - y, ez - z); n++;
  }
  void Tinv;
  return s / n;
};

// ---------- point-cloud basics ----------
function subsample(xyz, n, seed) {
  const N = xyz.length / 3; if (N <= n) return xyz.slice();
  const R = rng(seed), out = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { const j = Math.floor(R() * N); out[3 * i] = xyz[3 * j]; out[3 * i + 1] = xyz[3 * j + 1]; out[3 * i + 2] = xyz[3 * j + 2]; }
  return out;
}
Core.subsample = subsample;
function voxelDown(xyz, size) {
  const map = new Map(), N = xyz.length / 3;
  for (let i = 0; i < N; i++) {
    const k = Math.floor(xyz[3 * i] / size) * 73856093 ^ Math.floor(xyz[3 * i + 1] / size) * 19349663 ^ Math.floor(xyz[3 * i + 2] / size) * 83492791;
    const a = map.get(k);
    if (a) { a[0] += xyz[3 * i]; a[1] += xyz[3 * i + 1]; a[2] += xyz[3 * i + 2]; a[3]++; } else map.set(k, [xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2], 1]);
  }
  const out = new Float32Array(map.size * 3); let j = 0;
  for (const a of map.values()) { out[j++] = a[0] / a[3]; out[j++] = a[1] / a[3]; out[j++] = a[2] / a[3]; }
  return out;
}
Core.voxelDown = voxelDown;
// surface normals from the neighbours within `radius` (PCA, smallest eigenvector)
function normalsPCA(xyz, radius) {
  const N = xyz.length / 3, h = radius, grid = new Map(), nrm = new Float32Array(N * 3);
  const key = (a, b, c) => a * 73856093 ^ b * 19349663 ^ c * 83492791;
  for (let i = 0; i < N; i++) {
    const k = key(Math.floor(xyz[3 * i] / h), Math.floor(xyz[3 * i + 1] / h), Math.floor(xyz[3 * i + 2] / h));
    const a = grid.get(k); if (a) a.push(i); else grid.set(k, [i]);
  }
  const r2 = radius * radius;
  for (let i = 0; i < N; i++) {
    const x = xyz[3 * i], y = xyz[3 * i + 1], z = xyz[3 * i + 2], gx = Math.floor(x / h), gy = Math.floor(y / h), gz = Math.floor(z / h);
    let n = 0, sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, sxz = 0, syy = 0, syz = 0, szz = 0;
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
      const L = grid.get(key(gx + a, gy + b, gz + c)); if (!L) continue;
      for (const j of L) {
        const dx = xyz[3 * j] - x, dy = xyz[3 * j + 1] - y, dz = xyz[3 * j + 2] - z;
        if (dx * dx + dy * dy + dz * dz > r2) continue;
        n++; sx += dx; sy += dy; sz += dz; sxx += dx * dx; sxy += dx * dy; sxz += dx * dz; syy += dy * dy; syz += dy * dz; szz += dz * dz;
      }
    }
    if (n < 6) continue;
    const mx = sx / n, my = sy / n, mz = sz / n;
    const C = [sxx / n - mx * mx, sxy / n - mx * my, sxz / n - mx * mz, syy / n - my * my, syz / n - my * mz, szz / n - mz * mz];
    const v = smallestEig(C);
    nrm[3 * i] = v[0]; nrm[3 * i + 1] = v[1]; nrm[3 * i + 2] = v[2];
  }
  return nrm;
}
Core.normalsPCA = normalsPCA;
function smallestEig(c) {   // symmetric 3x3 [xx xy xz yy yz zz], Jacobi
  const a = [[c[0], c[1], c[2]], [c[1], c[3], c[4]], [c[2], c[4], c[5]]], v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 8; sweep++) {
    for (let p = 0; p < 2; p++) for (let q = p + 1; q < 3; q++) {
      if (Math.abs(a[p][q]) < 1e-12) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const cs = 1 / Math.sqrt(t * t + 1), sn = t * cs;
      for (let k = 0; k < 3; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = cs * akp - sn * akq; a[k][q] = sn * akp + cs * akq; }
      for (let k = 0; k < 3; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = cs * apk - sn * aqk; a[q][k] = sn * apk + cs * aqk; }
      for (let k = 0; k < 3; k++) { const vkp = v[k][p], vkq = v[k][q]; v[k][p] = cs * vkp - sn * vkq; v[k][q] = sn * vkp + cs * vkq; }
    }
  }
  let m = 0; if (a[1][1] < a[m][m]) m = 1; if (a[2][2] < a[m][m]) m = 2;
  return [v[0][m], v[1][m], v[2][m]];
}

// ---------- FFT (radix 2) ----------
function fft1(re, im, n, inv) {
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit;
    if (i < j) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = 2 * Math.PI / len * (inv ? 1 : -1), wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let j = 0; j < len / 2; j++) {
        const a = i + j, b = a + len / 2, xr = re[b] * cr - im[b] * ci, xi = re[b] * ci + im[b] * cr;
        re[b] = re[a] - xr; im[b] = im[a] - xi; re[a] += xr; im[a] += xi;
        const t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
      }
    }
  }
}
function fft2(re, im, G, inv) {
  const r = new Float64Array(G), i_ = new Float64Array(G);
  for (let y = 0; y < G; y++) { const o = y * G; for (let x = 0; x < G; x++) { r[x] = re[o + x]; i_[x] = im[o + x]; } fft1(r, i_, G, inv); for (let x = 0; x < G; x++) { re[o + x] = r[x]; im[o + x] = i_[x]; } }
  for (let x = 0; x < G; x++) { for (let y = 0; y < G; y++) { r[y] = re[y * G + x]; i_[y] = im[y * G + x]; } fft1(r, i_, G, inv); for (let y = 0; y < G; y++) { re[y * G + x] = r[y]; im[y * G + x] = i_[y]; } }
}
Core.fft2 = fft2;

// ---------- BIM-side samples used by the registration (complete design, known ahead) ----------
Core.bimModel = function (bim) {
  const soup = { tri: bim.tri, triElem: bim.triElem, n: bim.nT }, bvh = new BVH(soup);
  // area-weighted surface samples split into walls (vertical) and slabs (horizontal)
  const R = rng(5), walls = [], flats = [];
  for (let t = 0; t < bim.nT; t++) {
    const o = 9 * t, T = bim.tri, A = triArea(T, t);
    const ax = T[o + 3] - T[o], ay = T[o + 4] - T[o + 1], az = T[o + 5] - T[o + 2], bx = T[o + 6] - T[o], by = T[o + 7] - T[o + 1], bz = T[o + 8] - T[o + 2];
    const nz = Math.abs(ax * by - ay * bx) / (2 * A || 1);
    const dens = 6, k = A * dens, cnt = Math.floor(k) + (R() < k - Math.floor(k) ? 1 : 0);
    for (let i = 0; i < cnt; i++) {
      let u = R(), v = R(); if (u + v > 1) { u = 1 - u; v = 1 - v; }
      const x = T[o] + u * ax + v * bx, y = T[o + 1] + u * ay + v * by, z = T[o + 2] + u * az + v * bz;
      if (nz < 0.3) walls.push(x, y, z); else if (nz > 0.9) flats.push(z);
    }
  }
  return { bvh, walls: new Float32Array(walls), flatZ: new Float32Array(flats), center: bim.center, lo: bim.lo, hi: bim.hi };
};

// ---------- ICP: point-to-plane against the BIM surface, robust (Tukey) or plain ----------
function solve6(A, b) {   // Gaussian elimination with partial pivoting, A 6x6 (array of 36), b 6
  const n = 6, m = A.slice(), x = b.slice();
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(m[r * n + c]) > Math.abs(m[p * n + c])) p = r;
    if (Math.abs(m[p * n + c]) < 1e-12) return null;
    if (p !== c) { for (let k = 0; k < n; k++) { const t = m[c * n + k]; m[c * n + k] = m[p * n + k]; m[p * n + k] = t; } const t = x[c]; x[c] = x[p]; x[p] = t; }
    for (let r = c + 1; r < n; r++) { const f = m[r * n + c] / m[c * n + c]; for (let k = c; k < n; k++) m[r * n + k] -= f * m[c * n + k]; x[r] -= f * x[c]; }
  }
  for (let r = n - 1; r >= 0; r--) { let s = x[r]; for (let k = r + 1; k < n; k++) s -= m[r * n + k] * x[k]; x[r] = s / m[r * n + r]; }
  return x;
}
// one ICP iteration; returns { T (updated), inliers, rms }
Core.icpStep = function (model, src, T, maxDist, tukeyK) {
  const N = src.length / 3, A = new Array(36).fill(0), b = new Array(6).fill(0), bvh = model.bvh;
  let nIn = 0, sse = 0;
  for (let i = 0; i < N; i++) {
    const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2];
    const px = T[0] * x + T[1] * y + T[2] * z + T[3], py = T[4] * x + T[5] * y + T[6] * z + T[7], pz = T[8] * x + T[9] * y + T[10] * z + T[11];
    const d = bvh.closest(px, py, pz, maxDist, -1);
    if (d === Infinity) continue;
    const k = bvh.ct, nx = bvh.nrm[3 * k], ny = bvh.nrm[3 * k + 1], nz = bvh.nrm[3 * k + 2];
    const r = (px - bvh.cp[0]) * nx + (py - bvh.cp[1]) * ny + (pz - bvh.cp[2]) * nz;
    let w = 1;
    if (tukeyK) { const q = r / tukeyK; w = Math.abs(q) < 1 ? (1 - q * q) ** 2 : 0; }
    if (w <= 0) continue;
    const J = [py * nz - pz * ny, pz * nx - px * nz, px * ny - py * nx, nx, ny, nz];
    for (let a = 0; a < 6; a++) { b[a] -= w * J[a] * r; for (let c = a; c < 6; c++) A[a * 6 + c] += w * J[a] * J[c]; }
    nIn++; sse += r * r;
  }
  for (let a = 0; a < 6; a++) for (let c = 0; c < a; c++) A[a * 6 + c] = A[c * 6 + a];
  for (let a = 0; a < 6; a++) A[a * 7] += 1e-6;
  const x = nIn >= 6 ? solve6(A, b) : null;
  if (!x) return { T, inliers: nIn, rms: Infinity };
  const ang = Math.hypot(x[0], x[1], x[2]) * 180 / Math.PI;
  const dR = ang > 1e-9 ? M.rotAxis(x[0], x[1], x[2], ang) : M.eye();
  dR[3] = x[3]; dR[7] = x[4]; dR[11] = x[5];
  return { T: M.mul(dR, T), inliers: nIn, rms: Math.sqrt(sse / Math.max(nIn, 1)) };
};
// share of points within `dist` of the BIM after transform T
Core.fitShare = function (model, src, T, dist) {
  const N = src.length / 3; let n = 0;
  for (let i = 0; i < N; i++) {
    const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2];
    if (model.bvh.closest(T[0] * x + T[1] * y + T[2] * z + T[3], T[4] * x + T[5] * y + T[6] * z + T[7], T[8] * x + T[9] * y + T[10] * z + T[11], dist, -1) < Infinity) n++;
  }
  return n / N;
};

// Plain ICP (what the repo's icp_old does in spirit): start where the cloud is and pull every point
// to its nearest BIM surface. Records each iteration for playback.
Core.plainICP = function (model, xyz, opts) {
  opts = opts || {};
  const src = subsample(xyz, opts.n || 1500, 31), frames = [];
  // first move the scan's centre onto the design's centre (fixes the shift, not the heading)
  const N = xyz.length / 3, c = [0, 0, 0];
  for (let i = 0; i < N; i++) for (let k = 0; k < 3; k++) c[k] += xyz[3 * i + k] / N;
  let T = M.eye();
  if (opts.centre !== false) {
    T = M.trans(model.center[0] - c[0], model.center[1] - c[1], model.center[2] - c[2]);
    frames.push({ T, rms: NaN, inliers: 0, dist: 0, centroid: true });
  }
  // wide pull first, then the repo's tight 0.3 m correspondence distance
  for (const [dist, iters] of [[opts.maxDist || 2.0, 25], [0.3, 15]]) for (let it = 0; it < iters; it++) {
    const r = Core.icpStep(model, src, T, dist, 0);
    if (r.rms === Infinity || r.inliers < 50) break;   // too few matches: plain ICP is stuck where it is
    frames.push({ T: r.T, rms: r.rms, inliers: r.inliers / (src.length / 3), dist });
    const moved = Math.hypot(r.T[3] - T[3], r.T[7] - T[7], r.T[11] - T[11]);
    T = r.T;
    if (moved < 1e-4 && it > 3) break;
  }
  return { T, frames };
};

// ---------- the "plan" registration (src/registration.py), browser-sized ----------
Core.PLAN = { VOXEL: 0.2, NORMAL_R: 0.45, VERT_MAX_NZ: 0.3, HORZ_MIN_NZ: 0.9, Z_BIN: 0.05, N_Z: 3, Z_SEP: 0.5,
  RES: 0.4, G: 128, YAW_STEP: 2, BAND_PAD: 0.3, N_VERIFY: 8, DZ_STEP: 0.2, DZ_FINE: 0.05, DZ_RANGE: 6.0, DZ_DIST: 0.15,
  VERIFY_DIST: 0.1, N_FINAL: 3, TIGHT: 0.05, AMBIG: 0.9, DISTINCT_SHIFT: 2.0, DISTINCT_YAW: 5,
  ICP_SCHEDULE: [[1.0, 0.5], [0.4, 0.2], [0.15, 0.1]], ICP_ITERS: 6 };

// prep: downsample, normals, level, split. Pure function of the cloud.
Core.planPrep = function (xyz) {
  const P = Core.PLAN, pts = voxelDown(xyz, P.VOXEL), nrm = normalsPCA(pts, P.NORMAL_R), N = pts.length / 3;
  let mx = 0, my = 0, mz = 0, cnt = 0;
  for (let i = 0; i < N; i++) { const nz = nrm[3 * i + 2]; if (Math.abs(nz) > P.HORZ_MIN_NZ) { const s = nz > 0 ? 1 : -1; mx += s * nrm[3 * i]; my += s * nrm[3 * i + 1]; mz += s * nz; cnt++; } }
  const l = Math.hypot(mx, my, mz); mx /= l; my /= l; mz /= l;
  const tilt = Math.acos(Math.min(1, mz)) * 180 / Math.PI;
  let c = [0, 0, 0]; for (let i = 0; i < N; i++) for (let k = 0; k < 3; k++) c[k] += pts[3 * i + k] / N;
  const Rl = tilt > 1e-6 ? M.rotAxis(my, -mx, 0, tilt) : M.eye();   // axis = n x z
  const Tlev = M.about(Rl, c[0], c[1], c[2]);
  const lev = M.apply(Tlev, pts), ln = M.applyDir(Tlev, nrm);
  const cls = new Uint8Array(N);   // 1 wall, 2 slab/ground, 0 other
  for (let i = 0; i < N; i++) { const a = Math.abs(ln[3 * i + 2]); if (ln[3 * i] === 0 && ln[3 * i + 1] === 0 && ln[3 * i + 2] === 0) continue; cls[i] = a < P.VERT_MAX_NZ ? 1 : a > P.HORZ_MIN_NZ ? 2 : 0; }
  return { pts, nrm, lev, cls, Tlev, tilt, centroid: c };
};

// height candidates: correlate the histogram of horizontal-surface heights with the BIM's
Core.heightCandidates = function (model, prep) {
  const P = Core.PLAN, zb = [];
  for (let i = 0; i < prep.cls.length; i++) if (prep.cls[i] === 2) zb.push(prep.lev[3 * i + 2]);
  const lo = Math.min(model.lo[2], ...zb.slice(0, 1)) - 40, bin = P.Z_BIN;
  let cmin = Infinity, cmax = -Infinity; for (const z of zb) { cmin = Math.min(cmin, z); cmax = Math.max(cmax, z); }
  const nC = Math.ceil((cmax - cmin) / bin) + 1, hc = new Float64Array(nC);
  for (const z of zb) hc[Math.floor((z - cmin) / bin)]++;
  const bmin = model.lo[2] - 0.5, nB = Math.ceil((model.hi[2] + 0.5 - bmin) / bin) + 1, hb = new Float64Array(nB);
  for (const z of model.flatZ) hb[Math.floor((z - bmin) / bin)]++;
  for (let i = 0; i < nC; i++) hc[i] = Math.sqrt(hc[i]); for (let i = 0; i < nB; i++) hb[i] = Math.sqrt(hb[i]);
  // shift k: cloud bin i lands on BIM bin i + k ; dz = bmin + k*bin - cmin
  const scores = [];
  for (let k = -nC; k < nB; k++) { let s = 0; for (let i = 0; i < nC; i++) { const j = i + k; if (j >= 0 && j < nB) s += hc[i] * hb[j]; } scores.push({ dz: bmin + k * bin - cmin, s }); }
  const sorted = scores.slice().sort((a, b) => b.s - a.s), keep = [];
  for (const c of sorted) { if (keep.every(q => Math.abs(q.dz - c.dz) >= P.Z_SEP)) keep.push(c); if (keep.length >= P.N_Z) break; }
  void lo;
  return { cands: keep, curve: scores, hc, hb, cmin, bmin };
};

// top-down images: BIM walls in a height band, and the cloud's wall points turned by `yaw`
Core.bimRaster = function (model, zlo, zhi) {
  const P = Core.PLAN, G = P.G, img = new Float64Array(G * G), c = model.center, W = model.walls;
  for (let i = 0; i < W.length / 3; i++) {
    const z = W[3 * i + 2]; if (z < zlo || z > zhi) continue;
    const gx = Math.floor((W[3 * i] - c[0]) / P.RES) + G / 2, gy = Math.floor((W[3 * i + 1] - c[1]) / P.RES) + G / 2;
    if (gx >= 0 && gy >= 0 && gx < G && gy < G) img[gy * G + gx] = 1;
  }
  const dil = new Float64Array(G * G);   // one-pixel dilation, as registration.py does
  for (let y = 1; y < G - 1; y++) for (let x = 1; x < G - 1; x++) if (img[y * G + x]) for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) dil[(y + a) * G + x + b] = 1;
  return dil;
};
Core.cloudRaster = function (wallXY, cx, cy, yaw) {
  const P = Core.PLAN, G = P.G, img = new Float64Array(G * G), c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180);
  let n = 0;
  for (let i = 0; i < wallXY.length / 2; i++) {
    const x = wallXY[2 * i] - cx, y = wallXY[2 * i + 1] - cy, rx = c * x - s * y, ry = s * x + c * y;
    const gx = Math.floor(rx / P.RES) + G / 2, gy = Math.floor(ry / P.RES) + G / 2;
    if (gx >= 0 && gy >= 0 && gx < G && gy < G && !img[gy * G + gx]) { img[gy * G + gx] = 1; n++; }
  }
  return { img, n };
};

// The whole plan search as a generator: yields progress events so the page can animate each part.
// Final event: { phase: 'done', T, ok, ambiguity, tight, ... }.
Core.planSearch = function* (model, xyz) {
  const P = Core.PLAN, G = P.G;
  const prep = Core.planPrep(xyz);
  yield { phase: 'level', T: prep.Tlev, tilt: prep.tilt, prep };
  const hz = Core.heightCandidates(model, prep);
  yield { phase: 'height', hz };
  // wall points (levelled) for the floor-plan image
  const wxy = [];
  let wx = 0, wy = 0, zlo = Infinity, zhi = -Infinity;
  for (let i = 0; i < prep.cls.length; i++) {
    const z = prep.lev[3 * i + 2]; zlo = Math.min(zlo, z); zhi = Math.max(zhi, z);
    if (prep.cls[i] !== 1) continue; wxy.push(prep.lev[3 * i], prep.lev[3 * i + 1]); wx += prep.lev[3 * i]; wy += prep.lev[3 * i + 1];
  }
  const nW = wxy.length / 2; wx /= nW; wy /= nW;
  const wallXY = new Float32Array(wxy);
  const cands = [], yawCurve = [], bims = [];
  for (let h = 0; h < hz.cands.length; h++) {   // one BIM floor-plan image per height band, transformed once
    const dz = hz.cands[h].dz, B = Core.bimRaster(model, zlo + dz - P.BAND_PAD, zhi + dz + P.BAND_PAD);
    const Br = B.slice(), Bi = new Float64Array(G * G); fft2(Br, Bi, G, false);
    bims.push({ dz, B, Br, Bi });
  }
  const Pr = new Float64Array(G * G), Pi = new Float64Array(G * G);
  for (let yaw = 0; yaw < 360; yaw += P.YAW_STEP) {
    const { img, n } = Core.cloudRaster(wallXY, wx, wy, yaw);
    const Cr = img.slice(), Ci = new Float64Array(G * G); fft2(Cr, Ci, G, false);
    let bestHere = null;
    for (let h = 0; h < bims.length; h++) {
      const { dz, Br, Bi } = bims[h];
      for (let i = 0; i < G * G; i++) { const ar = Br[i], ai = Bi[i], br = Cr[i], bi = -Ci[i]; Pr[i] = ar * br - ai * bi; Pi[i] = ar * bi + ai * br; }
      fft2(Pr, Pi, G, true);
      let best = -1, bi_ = 0;
      for (let i = 0; i < G * G; i++) if (Pr[i] > best) { best = Pr[i]; bi_ = i; }
      let sx = bi_ % G, sy = Math.floor(bi_ / G); if (sx >= G / 2) sx -= G; if (sy >= G / 2) sy -= G;
      const score = best / (G * G) / Math.max(n, 1);
      // candidate pose: levelled cloud, turned by yaw about the wall centroid, centroid -> BIM centre + shift
      const tx = model.center[0] + sx * P.RES, ty = model.center[1] + sy * P.RES;
      const T2 = M.mul(M.trans(tx, ty, dz), M.mul(M.rotZ(yaw), M.trans(-wx, -wy, 0)));
      const cand = { yaw, h, dz, score, sx, sy, T: M.mul(T2, prep.Tlev) };
      cands.push(cand);
      if (!bestHere || score > bestHere.score) bestHere = cand;
    }
    yawCurve.push({ yaw, score: bestHere.score });
    yield { phase: 'sweep', yaw, score: bestHere.score, cand: bestHere, yawCurve, bimImg: bims[bestHere.h].B, cloudImg: img };
  }
  // distinct 2D candidates
  cands.sort((a, b) => b.score - a.score);
  const distinct = (a, b) => { const dy = Math.abs(((a.yaw - b.yaw) % 360 + 540) % 360 - 180); return dy > P.DISTINCT_YAW || Math.hypot(a.T[3] - b.T[3], a.T[7] - b.T[7]) > P.DISTINCT_SHIFT; };
  const top = [];
  for (const c of cands) { if (top.every(q => distinct(q, c))) top.push(c); if (top.length >= P.N_VERIFY) break; }
  yield { phase: 'candidates', top };
  // height sweep in 3D for each candidate, keep two peaks
  const probe = subsample(xyz, 300, 41), verified = [];
  for (let ci = 0; ci < top.length; ci++) {
    // coarse sweep over more than a storey either way, then a fine sweep round each kept peak
    // (+-0.15 m only, so a fine sweep cannot climb into the look-alike height one slab away)
    const c = top[ci], curve = [], at = d => Core.fitShare(model, probe, M.mul(M.trans(0, 0, d), c.T), P.DZ_DIST);
    for (let d = -P.DZ_RANGE; d <= P.DZ_RANGE + 1e-9; d += P.DZ_STEP) curve.push({ d, s: at(d) });
    const coarse = curve.slice().sort((a, b) => b.s - a.s), seeds = [];
    // 5 peaks, 0.25 m apart: a storey (~3 m) and a slab thickness (~0.3 m) both make look-alike heights
    for (const p of coarse) { if (seeds.every(q => Math.abs(q.d - p.d) >= 0.25)) seeds.push(p); if (seeds.length >= 5) break; }
    const kept = seeds.map(sd => { let b = sd; for (let d = sd.d - 0.15; d <= sd.d + 0.15 + 1e-9; d += P.DZ_FINE) { const s = at(d); if (s > b.s) b = { d, s }; } return b; });
    for (const p of kept) verified.push({ ...c, T: M.mul(M.trans(0, 0, p.d), c.T), dzFine: p.d, sweepScore: p.s });
    yield { phase: 'dz', ci, n: top.length, curve, cand: c, kept };
  }
  // verify in 3D: quick ICP, then share of points within VERIFY_DIST
  const vsrc = subsample(xyz, 500, 43);
  for (const v of verified) {
    let T = v.T;
    for (let it = 0; it < 6; it++) T = Core.icpStep(model, vsrc, T, it < 3 ? 0.6 : 0.3, it < 3 ? 0.5 : 0.25).T;
    v.Tv = T; v.verify = Core.fitShare(model, vsrc, T, P.VERIFY_DIST);
    yield { phase: 'verify', v };
  }
  verified.sort((a, b) => b.verify - a.verify);
  const differ = (A, B, shift) => Math.hypot(A[3] - B[3], A[7] - B[7], A[11] - B[11]) > shift || Math.abs(M.yawDeg(M.mul(A, M.inv(B)))) > P.DISTINCT_YAW;
  const finals = [];
  // 0.2 m: two answers one slab thickness (~0.3 m) apart must both get the full fit
  for (const v of verified) { if (finals.every(q => differ(q.Tv, v.Tv, 0.2))) finals.push(v); if (finals.length >= P.N_FINAL) break; }
  // full refinement: multi-scale robust point-to-plane ICP; rank by tight fit
  const fsrc = subsample(xyz, 1500, 47), tsrc = subsample(xyz, 3000, 53);
  for (let fi = 0; fi < finals.length; fi++) {
    const f = finals[fi];
    let T = f.Tv; f.frames = [];
    for (const [dist, k] of P.ICP_SCHEDULE) for (let it = 0; it < P.ICP_ITERS; it++) {
      const r = Core.icpStep(model, fsrc, T, dist, k); T = r.T; f.frames.push({ T, rms: r.rms, dist });
      yield { phase: 'icp', fi, nf: finals.length, T, rms: r.rms, dist, it };
    }
    f.Tf = T; f.tight = Core.fitShare(model, tsrc, T, P.TIGHT);
    yield { phase: 'refine', f, fi };
  }
  finals.sort((a, b) => b.tight - a.tight);
  // ambiguity: best tight fit vs the best CLEARLY DIFFERENT final pose (same answer twice is not a tie)
  const best = finals[0], second = finals.slice(1).find(f => differ(f.Tf, best.Tf, P.DISTINCT_SHIFT));
  const ambiguity = second ? second.tight / best.tight : 0;
  yield { phase: 'done', T: best.Tf, best, finals, ok: ambiguity < P.AMBIG && best.tight > 0.2 && prep.tilt < 2, ambiguity, tight: best.tight, tilt: prep.tilt, yawCurve };
};
Core.runPlan = function (model, xyz) { let last; for (const ev of Core.planSearch(model, xyz)) last = ev; return last; };

// ---------- matching (p4_match_bim.py) ----------
Core.MATCH = { MAX_DIST: 0.08, MARGIN: 0.04, MIN_POINTS: 8 };
Core.matchPoints = function (model, bim, xyzBim, from, to, assigned) {
  const A = Core.MATCH, bvh = model.bvh;
  for (let i = from; i < to; i++) {
    const x = xyzBim[3 * i], y = xyzBim[3 * i + 1], z = xyzBim[3 * i + 2];
    const d1 = bvh.closest(x, y, z, A.MAX_DIST, -1);
    if (d1 === Infinity) { assigned[i] = -1; continue; }
    const e1 = bvh.e[bvh.ct];
    const d2 = bvh.closest(x, y, z, d1 + A.MARGIN, e1);   // nearest DIFFERENT element
    assigned[i] = d2 === Infinity ? e1 : -2;               // -2: on a joint, ambiguous
  }
};
Core.aggregate = function (bim, xyzBim, assigned) {
  const nE = bim.nE, cnt = new Int32Array(nE), zlo = new Float32Array(nE).fill(Infinity), zhi = new Float32Array(nE).fill(-Infinity);
  for (let i = 0; i < assigned.length; i++) { const e = assigned[i]; if (e < 0) continue; cnt[e]++; const z = xyzBim[3 * i + 2]; if (z < zlo[e]) zlo[e] = z; if (z > zhi[e]) zhi[e] = z; }
  const cov = new Float32Array(nE), det = new Uint8Array(nE);
  for (let e = 0; e < nE; e++) {
    if (cnt[e] < Core.MATCH.MIN_POINTS) continue;
    det[e] = 1; const h = bim.elem.zmax[e] - bim.elem.zmin[e];
    cov[e] = h > 1e-6 ? Math.min(1, (zhi[e] - zlo[e]) / h) : 1;
  }
  return { cnt, cov, det };
};

// ---------- visibility (p4_visibility.py): would the cameras have seen it if it were built? ----------
// rays come from the scan, in the BIM frame via the ESTIMATED registration.
Core.Visibility = function (bim, model, det, cams, rayView, rayDirBim) {
  this.occ = new BVH(Core.subsetTris(bim, e => det[e] === 1));
  this.cand = new BVH(Core.subsetTris(bim, e => det[e] !== 1));
  this.model = model; this.cams = cams; this.rayView = rayView; this.rayDir = rayDirBim; this.next = 0;
  this.stride = Core.VIS_STRIDE;   // test every n-th scan ray, count each n times
  this.expUndet = new Float32Array(bim.nE); this.expDet = new Float32Array(bim.nE);
};
Core.VIS_STRIDE = 3;
Core.Visibility.prototype.step = function (k) {
  const keepP = (1 - Core.SCAN.DROPOUT) * this.stride, A = Core.MATCH, model = this.model, sample = [];
  const end = Math.min(this.next + k * this.stride, this.rayView.length);
  for (let i = this.next; i < end; i += this.stride) {
    const [ox, oy, oz] = this.cams[this.rayView[i]], dx = this.rayDir[3 * i], dy = this.rayDir[3 * i + 1], dz = this.rayDir[3 * i + 2];
    const tOcc = this.occ.intersect(ox, oy, oz, dx, dy, dz, 80);
    if (tOcc < Infinity) {   // first hit on a detected element: does it land somewhere assignable?
      const e = this.occ.e[this.occ.hitTri], hx = ox + tOcc * dx, hy = oy + tOcc * dy, hz = oz + tOcc * dz;
      if (model.bvh.closest(hx, hy, hz, A.MARGIN, e) === Infinity) this.expDet[e] += keepP;
    }
    let first = -1, firstT = Infinity;
    this.cand.allHits(ox, oy, oz, dx, dy, dz, tOcc, (t, tri) => {
      const e = this.cand.e[tri], hx = ox + t * dx, hy = oy + t * dy, hz = oz + t * dz;
      if (model.bvh.closest(hx, hy, hz, A.MARGIN, e) === Infinity) this.expUndet[e] += keepP;
      if (t < firstT) { firstT = t; first = e; }
    });
    if ((i / this.stride) % 23 === 0) sample.push({ o: [ox, oy, oz], d: [dx, dy, dz], tOcc, tCand: firstT, cand: first });
  }
  this.next = end;
  return { done: this.next >= this.rayView.length, sample, frac: this.next / this.rayView.length };
};

// ---------- scoring (p7_score.py) ----------
Core.SCORE = { COMPLETED: 0.5, PARTIAL: 0.15, VISIBLE_MIN_EXPECTED: 30, DETECT_MIN_FRACTION: 0.1 };
// status codes: 0 not_started, 1 partial, 2 completed, 3 uncertain
Core.score = function (bim, agg, vis) {
  const S = Core.SCORE, nE = bim.nE, status = new Uint8Array(nE), c = new Float32Array(nE);
  let nRejected = 0;
  for (let e = 0; e < nE; e++) {
    let detected = agg.det[e] === 1, wellCovered = vis.expUndet[e] >= S.VISIBLE_MIN_EXPECTED;
    if (detected && agg.cnt[e] < S.DETECT_MIN_FRACTION * vis.expDet[e]) { detected = false; nRejected++; if (vis.expDet[e] >= S.VISIBLE_MIN_EXPECTED) wellCovered = true; }
    if (detected && agg.cov[e] >= S.COMPLETED) { status[e] = 2; c[e] = 1; }
    else if (detected && agg.cov[e] >= S.PARTIAL) { status[e] = 1; c[e] = agg.cov[e]; }
    else if (wellCovered) { status[e] = 0; c[e] = 0; }
    else status[e] = 3;
  }
  let sc = 0, n = 0, sv = 0, vsum = 0;
  for (let e = 0; e < nE; e++) if (status[e] !== 3) { sc += c[e]; n++; sv += c[e] * bim.elem.vol[e]; vsum += bim.elem.vol[e]; }
  return { status, c, nRejected, progressCount: 100 * sc / n, progressVolume: 100 * sv / vsum, nScored: n };
};
// ---------- simple clutter removal by colour (color_registration.py material_fit / MIN_FIT) ----------
// Uses the CATALOGUE colours (what the design says a material looks like), never the renderer's table.
Core.CATALOGUE_RGB = {
  brick_brown: [150, 85, 62], brick_yellow: [196, 160, 104], calcium_silicate: [205, 203, 196], aerated_concrete: [215, 213, 205],
  gypsum_block: [222, 220, 214], concrete_prefab: [182, 180, 174], concrete_cast: [168, 166, 160], screed: [158, 154, 146],
  steel_dark: [62, 66, 72], steel_primer: [140, 68, 52], wood: [150, 104, 64], plywood: [196, 164, 118],
  board_white: [232, 230, 224], zinc: [150, 155, 158], tile: [120, 116, 110], insulation: [226, 214, 150],
};
Core.COLOR = { CHROMA_SIGMA: 0.04, MIN_FIT: 0.05, DARK_SUM: 90 };
// keep[i] = 1 when point i's colour fits at least one design material
Core.colorFilter = function (rgb) {
  const C = Core.COLOR, cat = Object.values(Core.CATALOGUE_RGB).map(c => { const s = c[0] + c[1] + c[2]; return [c[0] / s, c[1] / s]; });
  const n = rgb.length / 3, keep = new Uint8Array(n), k2 = 2 * C.CHROMA_SIGMA ** 2, thr = -Math.log(C.MIN_FIT) * k2;
  for (let i = 0; i < n; i++) {
    const r = rgb[3 * i], g = rgb[3 * i + 1], b = rgb[3 * i + 2], s = r + g + b;
    if (s < C.DARK_SUM) { keep[i] = 1; continue; }   // too dark to tell the hue
    const x = r / (s + 1e-6), y = g / (s + 1e-6);
    keep[i] = cat.some(c => (x - c[0]) ** 2 + (y - c[1]) ** 2 <= thr) ? 1 : 0;
  }
  return keep;
};

// ---------- error bars (p7_score.py evidence_bin + calibrated, table from r5_calibrate.py) ----------
Core.R5 = { Z90: 1.645, MIN_BIN_N: 20, FRONT_MIN_COMPLETED: 10 };
const FAMILY = ['column', 'beam', 'slab', 'wall'];
Core.calibrated = function (bim, sc, agg, vis, calib, weighting) {
  const R = Core.R5, by = new Map(calib.map(r => [r.bin, r])), NAMES = ['not_started', 'partial', 'completed', 'uncertain'];
  const perStorey = new Map();
  for (let e = 0; e < bim.nE; e++) if (sc.status[e] === 2) perStorey.set(bim.elem.storey[e], (perStorey.get(bim.elem.storey[e]) || 0) + 1);
  let front = null;
  for (const [st, n] of perStorey) if (n >= R.FRONT_MIN_COMPLETED && (front === null || st > front)) front = st;
  let wsum = 0;
  const rows = [];
  for (let e = 0; e < bim.nE; e++) {
    const s = sc.status[e]; if (s === 3) continue;
    const det = agg.det[e] === 1 && s !== 0, n = det ? agg.cnt[e] : 0, exp = det ? vis.expDet[e] : vis.expUndet[e];
    const pos = front === null || bim.elem.storey[e] > front ? 'above' : bim.elem.storey[e] === front ? 'front' : 'below';
    let band;
    if (s === 0) band = exp < 100 ? 'e<100' : exp < 300 ? 'e<300' : 'e>=300';
    else if (exp <= 0) band = 'noexp';
    else { const r = n / exp; band = r < 0.5 ? 'r<0.5' : r < 2 ? 'r<2' : 'r>=2'; }
    const bin = `${NAMES[s]}/${pos}/${FAMILY[bim.elem.type[e]]}/${band}`;
    let key = bin;
    for (const k of [bin, bin.split('/').slice(0, 2).join('/'), NAMES[s]]) { key = k; const r = by.get(k); if (r && (r.n >= R.MIN_BIN_N || !k.includes('/'))) break; }
    const row = by.get(key) || { bias: 0, var: 0, se: 0 }, w = weighting === 'volume' ? bim.elem.vol[e] : 1;
    rows.push({ w, c: sc.c[e], row, key }); wsum += w;
  }
  let est = 0, spread = 0; const shared = new Map();
  for (const r of rows) {
    const w = r.w / wsum;
    est += w * Math.min(1, Math.max(0, r.c + r.row.bias)); spread += w * w * r.row.var;
    shared.set(r.key, (shared.get(r.key) || 0) + w * r.row.se);
  }
  let sh = 0; for (const v of shared.values()) sh += v * v;
  const st = by.get(`_stage_${weighting}`), half = 100 * R.Z90 * Math.sqrt(spread + sh + (st ? st.var : 0));
  return { est: 100 * est, lo: 100 * est - half, hi: 100 * est + half };
};

// ground truth over the SAME non-uncertain set (eval only); on the busy site late and misplaced
// elements count as not built (p7_score.py)
Core.truthProgress = function (bim, s, status, ev) {
  let sc = 0, n = 0, sv = 0, vsum = 0, nMissed = 0, nFalse = 0;
  const late = ev ? ev.late : new Set(), moved = ev ? ev.misplaced : new Set();
  for (let e = 0; e < bim.nE; e++) {
    const v = bim.stateAt(s, e), g = late.has(e) || moved.has(e) ? 0 : v === 255 ? 1 : v / 254;
    if (status[e] === 3) continue;
    sc += g; n++; sv += g * bim.elem.vol[e]; vsum += bim.elem.vol[e];
    if (g > 0 && status[e] === 0) nMissed++;
    if (g === 0 && (status[e] === 1 || status[e] === 2)) nFalse++;
  }
  return { progressCount: 100 * sc / n, progressVolume: 100 * sv / vsum, nMissed, nFalse };
};

if (typeof module !== 'undefined' && module.exports) module.exports = Core; else root.Core = Core;
})(typeof window !== 'undefined' ? window : globalThis);
