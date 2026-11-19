import { BicycleVehicle } from './dynamics.js';

const $ = (id) => document.getElementById(id);
const vehicle = new BicycleVehicle();
const canvas = $('simulation');
const ctx = canvas.getContext('2d', { alpha: false });
const input = { steer: 0, throttle: 0, brake: 0, reverse: false };
const held = new Set();
const trail = [];
const cones = [ [-7,-8], [-7,-4], [-7,0], [-7,4], [-7,8], [16,-8], [16,8], [25,-8], [25,8] ];
let assistEnabled = true;
let mu = 0.88;
let dragSteer = false;
let viewportWidth = 1, viewportHeight = 1, dpr = 1;
let prevTime = 0, accumulator = 0, trailCounter = 0;
const physicsStep = 0.02;

function resize() {
  const box = canvas.getBoundingClientRect();
  viewportWidth = box.width;
  viewportHeight = box.height;
  dpr = Math.min(window.devicePixelRatio || 1, 2); // raster budget on mobile
  canvas.width = Math.round(box.width * dpr);
  canvas.height = Math.round(box.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize, { passive: true });
resize();

$('reset').addEventListener('click', () => { vehicle.reset(); trail.length = 0; });
$('clearTrail').addEventListener('click', () => { trail.length = 0; });
$('assist').addEventListener('change', (e) => { assistEnabled = e.target.checked; $('assistBadge').textContent = assistEnabled ? 'ASSIST ON' : 'ASSIST OFF'; });
$('mu').addEventListener('input', (e) => {
  mu = Number(e.target.value);
  $('muValue').textContent = mu.toFixed(2);
  $('roadStatus').textContent = `μ = ${mu.toFixed(2)}`;
  $('roadBadge').textContent = mu >= 0.65 ? 'DRY ROAD' : 'WET ROAD';
});
function setGear(reverse) {
  input.reverse = reverse;
  $('drive').classList.toggle('selected', !reverse);
  $('reverse').classList.toggle('selected', reverse);
  $('gear').textContent = reverse ? 'R' : 'D';
}
$('drive').addEventListener('click', () => setGear(false));
$('reverse').addEventListener('click', () => setGear(true));

function attachHold(id, key) {
  const el = $(id);
  const up = (e) => { input[key] = 0; el.classList.remove('pressed'); try { el.releasePointerCapture(e.pointerId); } catch { /* already released */ } };
  el.addEventListener('pointerdown', (e) => { e.preventDefault(); input[key] = 1; el.classList.add('pressed'); el.setPointerCapture(e.pointerId); });
  el.addEventListener('pointerup', up);
  el.addEventListener('pointercancel', up);
  el.addEventListener('lostpointercapture', () => { input[key] = 0; el.classList.remove('pressed'); });
}
attachHold('throttle', 'throttle');
attachHold('brake', 'brake');

const steeringPad = $('steeringPad');
function steerFromPointer(e) {
  const r = steeringPad.getBoundingClientRect();
  input.steer = Math.max(-1, Math.min(1, (e.clientX - r.left - r.width / 2) / (r.width * 0.38)));
  updateSteeringVisual();
}
function updateSteeringVisual() {
  $('steeringWheel').style.setProperty('--wheel-angle', `${input.steer * 125}deg`);
  steeringPad.setAttribute('aria-valuenow', String(Math.round(input.steer * 100)));
}
steeringPad.addEventListener('pointerdown', (e) => { e.preventDefault(); dragSteer = true; steeringPad.setPointerCapture(e.pointerId); steerFromPointer(e); });
steeringPad.addEventListener('pointermove', (e) => { if (dragSteer) steerFromPointer(e); });
function releaseSteer() { dragSteer = false; input.steer = 0; updateSteeringVisual(); }
steeringPad.addEventListener('pointerup', releaseSteer);
steeringPad.addEventListener('pointercancel', releaseSteer);
steeringPad.addEventListener('lostpointercapture', releaseSteer);
steeringPad.addEventListener('keydown', e => { if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') { e.preventDefault(); input.steer = Math.max(-1, Math.min(1, input.steer + (e.key === 'ArrowLeft' ? -0.15 : 0.15))); updateSteeringVisual(); } });

const relevant = new Set(['KeyW','KeyA','KeyS','KeyD','KeyR','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space']);
window.addEventListener('keydown', (e) => { if (!relevant.has(e.code)) return; if (e.target instanceof HTMLInputElement) return; e.preventDefault(); if (e.code === 'KeyR' && !held.has(e.code)) setGear(!input.reverse); held.add(e.code); });
window.addEventListener('keyup', (e) => held.delete(e.code));
window.addEventListener('blur', () => { held.clear(); input.throttle = 0; input.brake = 0; releaseSteer(); });
function updateKeys(dt) {
  if (!dragSteer) {
    const left = held.has('KeyA') || held.has('ArrowLeft');
    const right = held.has('KeyD') || held.has('ArrowRight');
    if (left !== right) input.steer = Math.max(-1, Math.min(1, input.steer + (right ? 2.2 : -2.2) * dt));
    else if (!(document.activeElement === steeringPad)) input.steer += (0 - input.steer) * Math.min(1, dt * 4.5);
    updateSteeringVisual();
  }
  if (held.has('KeyW') || held.has('ArrowUp')) input.throttle = 1;
  else if (!$('throttle').classList.contains('pressed')) input.throttle = 0;
  if (held.has('KeyS') || held.has('ArrowDown') || held.has('Space')) input.brake = 1;
  else if (!$('brake').classList.contains('pressed')) input.brake = 0;
}

function roundedRect(x, y, w, h, r, fill) {
  ctx.beginPath();ctx.roundRect(x,y,w,h,r);ctx.fillStyle=fill;ctx.fill();
}
function drawWorld() {
  const w = viewportWidth, h = viewportHeight;
  ctx.fillStyle = '#24342f'; ctx.fillRect(0, 0, w, h);
  const scale = Math.min(13, Math.max(6, w / 65));
  const camX = vehicle.x * .66, camY = vehicle.y * .66;
  const tx = w / 2 - camX * scale, ty = h / 2 - camY * scale;
  ctx.save(); ctx.translate(tx, ty); ctx.scale(scale, scale);
  // The parking lot and 2m ground grid are drawn entirely as a 2D static visualization.
  ctx.fillStyle = '#30433d'; ctx.fillRect(-80, -80, 160, 160);
  ctx.strokeStyle = '#3a5047'; ctx.lineWidth = 0.035;
  ctx.beginPath();for(let x=-80;x<=80;x+=2){ctx.moveTo(x,-80);ctx.lineTo(x,80);}for(let y=-80;y<=80;y+=2){ctx.moveTo(-80,y);ctx.lineTo(80,y);}ctx.stroke();
  ctx.fillStyle = '#394e45'; ctx.fillRect(-12,-13,45,26);
  ctx.strokeStyle = '#c4d1c1';ctx.lineWidth = .11;
  for(let x=-10;x<=30;x+=3.5){ctx.beginPath();ctx.moveTo(x,-12);ctx.lineTo(x,-6);ctx.moveTo(x,6);ctx.lineTo(x,12);ctx.stroke();}
  for(let y of [-12,-6,6,12]){ctx.beginPath();ctx.moveTo(-10,y);ctx.lineTo(30,y);ctx.stroke();}
  ctx.setLineDash([.8,.55]);ctx.strokeStyle='#b3c7b6';ctx.lineWidth=.06;ctx.beginPath();ctx.moveTo(-65,0);ctx.lineTo(70,0);ctx.stroke();ctx.setLineDash([]);
  for (const [x,y] of cones) {ctx.fillStyle='#edaa68';ctx.beginPath();ctx.arc(x,y,.3,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#fff2d7';ctx.lineWidth=.07;ctx.beginPath();ctx.arc(x,y,.18,0,Math.PI*2);ctx.stroke();}
  if(trail.length>1){ctx.beginPath();ctx.moveTo(trail[0][0],trail[0][1]);for(let i=1;i<trail.length;i++)ctx.lineTo(trail[i][0],trail[i][1]);ctx.strokeStyle='#7fe4b5';ctx.lineWidth=.14;ctx.stroke();}
  // A live wheelbase-scale indicator to demonstrate true scene coordinates.
  ctx.strokeStyle='#88a99d';ctx.lineWidth=.08;ctx.beginPath();ctx.moveTo(-30,17);ctx.lineTo(-20,17);ctx.stroke();
  ctx.fillStyle='#9fbdaf';ctx.font='.9px sans-serif';ctx.fillText('10 m',-26,19);
  drawCar();
  ctx.restore();
}
function drawCar() {
  ctx.save();ctx.translate(vehicle.x,vehicle.y);ctx.rotate(vehicle.heading);
  // Cars point along local +X; body dimensions roughly mid-size sedan.
  ctx.fillStyle='#111d20'; ctx.shadowColor='#071813';ctx.shadowBlur=.8;ctx.shadowOffsetY=.15;
  roundedRect(-2.25,-.98,4.5,1.96,.38,'#071713');ctx.shadowBlur=0;ctx.shadowOffsetY=0;
  roundedRect(-2.2,-.9,4.4,1.8,.34,'#d5eee2');
  roundedRect(-.85,-.81,1.55,1.62,.22,'#567e72');
  roundedRect(-.74,-.66,1.31,1.32,.2,'#37564e');
  ctx.fillStyle='#9ac1b0';ctx.fillRect(1.92,-.68,.18,.35);ctx.fillRect(1.92,.33,.18,.35);
  ctx.fillStyle='#e95f60';ctx.fillRect(-2.17,-.7,.14,.35);ctx.fillRect(-2.17,.35,.14,.35);
  const wheelAt = [[1.15,-1.02,true],[1.15,1.02,true],[-1.55,-1.02,false],[-1.55,1.02,false]];
  for(const [x,y,front] of wheelAt){ctx.save();ctx.translate(x,y);if(front)ctx.rotate(vehicle.steerRad);roundedRect(-.35,-.12,.7,.24,.05,'#0d1515');ctx.restore();}
  ctx.fillStyle='#bddccd';ctx.beginPath();ctx.moveTo(1.75,0);ctx.lineTo(1.25,-.42);ctx.lineTo(1.25,.42);ctx.closePath();ctx.fill();
  ctx.restore();
}
function renderHud() {
  $('speed').textContent = (Math.abs(vehicle.u) * 3.6).toFixed(1);
  $('yawRate').innerHTML = `${(vehicle.yawRate * 180 / Math.PI).toFixed(1)}<span>°/s</span>`;
  $('beta').innerHTML = `${(vehicle.betaRad * 180 / Math.PI).toFixed(1)}<span>°</span>`;
  $('frontFy').innerHTML = `${Math.round(vehicle.lastFyFrontN)}<span>N</span>`;
  $('rearFy').innerHTML = `${Math.round(vehicle.lastFyRearN)}<span>N</span>`;
  $('tractionText').textContent = `${Math.round(vehicle.traction*100)}%`;
  $('tractionFill').style.width = `${Math.round(vehicle.traction*100)}%`;
}
function frame(time) {
  if (!prevTime) prevTime = time;
  // Cap catch-up to avoid spiral-of-death after tab suspension / device slowdown.
  const elapsed = Math.min((time-prevTime)/1000, 0.12);
  prevTime = time;accumulator += elapsed;
  let steps = 0;
  while(accumulator >= physicsStep && steps < 5){
    updateKeys(physicsStep);
    vehicle.step(input,physicsStep,mu,assistEnabled);
    if(++trailCounter % 3===0){trail.push([vehicle.x,vehicle.y]);if(trail.length>650)trail.shift();}
    accumulator -= physicsStep;steps++;
  }
  if(steps===5)accumulator = 0;
  drawWorld();renderHud();requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
