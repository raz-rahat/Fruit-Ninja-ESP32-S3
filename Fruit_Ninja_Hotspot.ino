#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// =====================================================
// WIFI (Access Point / Hotspot mode)
// =====================================================
//
// The ESP32 creates its OWN WiFi network instead of joining your
// router. No home router needed - connect your phone/PC's WiFi
// directly to this network, and the game is always reachable at
// the fixed address http://192.168.4.1 (ESP32's standard AP IP).

const char* AP_SSID     = "FruitNinja";  // WiFi name the phone/PC will see
const char* AP_PASSWORD = "12345678";         // must be 8+ characters (WPA2). Use "" for an open network.

// =====================================================
// I2C
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 9

#define MPU_ADDR 0x68

// =====================================================
// SERVER
// =====================================================

WebServer server(80);
WebSocketsServer webSocket(81);

// =====================================================
// MPU6500 REGISTERS
// =====================================================

#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D
#define REG_ACCEL_XOUT_H     0x3B
#define REG_TEMP_OUT_H       0x41
#define REG_GYRO_XOUT_H      0x43
#define REG_PWR_MGMT_1       0x6B
#define REG_PWR_MGMT_2       0x6C
#define REG_WHO_AM_I        0x75

// =====================================================
// SENSOR DATA
// =====================================================

float ax, ay, az;
float gx, gy, gz;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float accelOffsetX = 0;
float accelOffsetY = 0;
float accelOffsetZ = 0;

// Filtered values
float fgx = 0;
float fgy = 0;
float fgz = 0;

float fax = 0;
float fay = 0;
float faz = 0;

const float FILTER_ALPHA = 0.12f;  // More filtering = less jitter

// =====================================================
// GESTURE
// =====================================================

bool slashActive = false;

unsigned long slashStart = 0;
unsigned long lastSlash = 0;

float slashDX = 0;
float slashDY = 0;
float slashPeak = 0;

const float SLASH_START = 130.0f;
const float SLASH_END   = 45.0f;

const unsigned long MAX_SLASH_TIME = 320;
const unsigned long SLASH_COOLDOWN = 140;

// =====================================================
// CALIBRATION
// =====================================================

bool calibrated = false;

// =====================================================
// I2C FUNCTIONS
// =====================================================

void writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  Wire.requestFrom(MPU_ADDR, (uint8_t)1);

  if (Wire.available())
    return Wire.read();

  return 0xFF;
}

// =====================================================
// MPU6500 INIT
// =====================================================

bool initMPU()
{
  uint8_t who = readReg(REG_WHO_AM_I);

  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x70)
  {
    Serial.println("MPU6500 not detected!");
    return false;
  }

  // Wake up
  writeReg(REG_PWR_MGMT_1, 0x00);
  delay(100);

  // Sample rate
  writeReg(REG_SMPLRT_DIV, 0x04);

  // DLPF
  writeReg(REG_CONFIG, 0x03);

  // Gyro ±1000 DPS
  writeReg(REG_GYRO_CONFIG, 0x10);

  // Accelerometer ±8G
  writeReg(REG_ACCEL_CONFIG, 0x10);

  // Accelerometer DLPF
  writeReg(REG_ACCEL_CONFIG2, 0x03);

  Serial.println("MPU6500 initialized.");
  return true;
}

// =====================================================
// READ MPU6500
// =====================================================

bool readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0)
    return false;

  uint8_t received = Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  if (received != 14)
    return false;

  int16_t rawAX = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawAY = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawAZ = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawTemp = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawGX = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawGY = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t rawGZ = ((int16_t)Wire.read() << 8) | Wire.read();

  // ±8G = 4096 LSB/G
  ax = ((float)rawAX / 4096.0f);
  ay = ((float)rawAY / 4096.0f);
  az = ((float)rawAZ / 4096.0f);

  // ±1000 DPS = 32.8 LSB/deg/s
  gx = ((float)rawGX / 32.8f);
  gy = ((float)rawGY / 32.8f);
  gz = ((float)rawGZ / 32.8f);

  return true;
}

// =====================================================
// CALIBRATION
// =====================================================

void calibrateSensor()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println("CALIBRATION");
  Serial.println("==============================");

  webSocket.broadcastTXT("{\"type\":\"calibration\",\"message\":\"Calibrating...\"}");

  const int samples = 800;

  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  int valid = 0;

  for (int i = 0; i < samples; i++)
  {
    if (readMPU())
    {
      sumGX += gx; sumGY += gy; sumGZ += gz;
      sumAX += ax; sumAY += ay; sumAZ += az;
      valid++;
    }
    delay(3);
  }

  if (valid == 0)
  {
    Serial.println("Calibration failed!");
    return;
  }

  gyroOffsetX = sumGX / valid;
  gyroOffsetY = sumGY / valid;
  gyroOffsetZ = sumGZ / valid;

  accelOffsetX = sumAX / valid;
  accelOffsetY = sumAY / valid;
  accelOffsetZ = (sumAZ / valid) - 1.0f;

  calibrated = true;

  Serial.println("Calibration complete.");
  webSocket.broadcastTXT("{\"type\":\"calibration\",\"message\":\"Calibrated!\"}");
}

// =====================================================
// WEBSOCKET
// =====================================================

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length)
{
  if (type == WStype_CONNECTED)
  {
    Serial.println("PC connected to WebSocket.");
    webSocket.sendTXT(num, "{\"type\":\"status\",\"message\":\"connected\"}");
  }

  if (type == WStype_DISCONNECTED)
  {
    Serial.println("PC disconnected.");
  }

  if (type == WStype_TEXT)
  {
    String command = String((char*)payload);
    if (command == "CALIBRATE")
    {
      calibrateSensor();
    }
  }
}

// =====================================================
// WEB PAGE
// =====================================================

const char GAME_HTML[] PROGMEM = R"HTML(

<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 WiFi Fruit Ninja</title>

<style>
*{box-sizing:border-box;}
html,body{
margin:0;padding:0;
width:100%;height:100%;
overflow:hidden;
font-family:Arial,sans-serif;
background:#030712;
}
canvas{
position:fixed;
left:0;top:0;
width:100%;height:100%;
}
#titlebar{
position:fixed;
top:0;left:0;right:0;
height:54px;
z-index:12;
display:flex;
align-items:center;
justify-content:center;
background:rgba(3,7,18,.94);
backdrop-filter:blur(12px);
border-bottom:1px solid rgba(255,255,255,.08);
}
#gameTitle{
font-family:'Segoe UI','Trebuchet MS',Arial,sans-serif;
font-size:24px;
font-weight:900;
letter-spacing:1px;
background:linear-gradient(90deg,#ff416c,#ff9800,#ffee58,#66bb6a,#26c6da,#7e57c2,#ff416c);
background-size:400% 100%;
-webkit-background-clip:text;
background-clip:text;
color:transparent;
animation:titleShine 6s linear infinite;
text-shadow:0 2px 14px rgba(255,255,255,.18);
white-space:nowrap;
}
#gameTitle .byline{
font-size:13px;
font-weight:700;
margin-left:8px;
opacity:.92;
}
@keyframes titleShine{
0%{background-position:0% 50%;}
100%{background-position:400% 50%;}
}
#countdownBox{
position:absolute;
right:16px;
top:50%;
transform:translateY(-50%);
font-size:26px;
font-weight:900;
font-variant-numeric:tabular-nums;
min-width:58px;
text-align:center;
color:#ffca28;
background:rgba(255,255,255,.08);
padding:4px 14px;
border-radius:10px;
}
#countdownBox.warn{
color:#ff1744;
animation:pulseWarn .5s infinite alternate;
}
@keyframes pulseWarn{
from{transform:translateY(-50%) scale(1);text-shadow:0 0 6px #ff1744;}
to{transform:translateY(-50%) scale(1.18);text-shadow:0 0 22px #ff1744;}
}
#hud{
position:fixed;
top:54px;left:0;right:0;
height:72px;
z-index:10;
display:flex;
align-items:center;
justify-content:space-between;
padding:0 20px;
background:rgba(3,7,18,.86);
backdrop-filter:blur(12px);
color:white;
border-bottom:1px solid rgba(255,255,255,.08);
}
#gameover{
display:none;
position:fixed;
left:0;top:0;right:0;bottom:0;
z-index:30;
flex-direction:column;
align-items:center;
justify-content:center;
gap:10px;
background:rgba(4,0,0,.82);
text-align:center;
}
#dangerSign{
font-size:64px;
color:#ff1744;
animation:blinkDanger .6s infinite alternate;
}
@keyframes blinkDanger{
from{opacity:.5;text-shadow:0 0 10px #ff1744;}
to{opacity:1;text-shadow:0 0 40px #ff1744;}
}
#gameoverText{
font-size:56px;
font-weight:900;
letter-spacing:3px;
color:#ff1744;
text-shadow:0 0 25px rgba(255,23,68,.85);
}
#finalScoreLabel{
font-size:16px;
color:#b0bec5;
letter-spacing:2px;
text-transform:uppercase;
margin-top:6px;
}
#finalScore{
font-size:48px;
font-weight:900;
color:white;
}
#playAgainBtn{
margin-top:18px;
border:none;
border-radius:10px;
padding:12px 26px;
font-weight:bold;
font-size:16px;
color:white;
cursor:pointer;
background:#d32f2f;
}
#stats{
display:flex;
gap:28px;
font-size:20px;
font-weight:bold;
}
#controls{
display:flex;
gap:8px;
align-items:center;
}
button{
border:none;
border-radius:8px;
padding:10px 15px;
font-weight:bold;
color:white;
cursor:pointer;
background:#1976d2;
}
#reset{
background:#455a64;
}
#status{
font-size:13px;
padding:8px 12px;
border-radius:7px;
background:rgba(255,255,255,.08);
color:#ffca28;
}
#debug{
position:fixed;
bottom:12px;
left:12px;
z-index:10;
padding:8px 10px;
border-radius:7px;
font:12px monospace;
color:#b0bec5;
background:rgba(0,0,0,.45);
}
#message{
position:fixed;
left:50%;
top:50%;
transform:translate(-50%,-50%);
z-index:8;
text-align:center;
color:white;
font-size:30px;
font-weight:bold;
pointer-events:none;
text-shadow:0 3px 20px black;
}

/* COUNTDOWN OVERLAY - Full screen countdown */
#countdownOverlay{
position:fixed;
left:0;top:0;right:0;bottom:0;
z-index:50;
display:flex;
flex-direction:column;
align-items:center;
justify-content:center;
background:rgba(0,0,0,.92);
color:white;
font-size:120px;
font-weight:900;
text-shadow:0 0 60px rgba(255,65,108,.5);
pointer-events:none;
transition:opacity 0.3s;
}
#countdownOverlay.hidden{
opacity:0;
pointer-events:none;
}
#countdownSub{
font-size:24px;
font-weight:400;
color:#ffca28;
margin-top:10px;
letter-spacing:4px;
}
</style>
</head>

<body>

<div id="titlebar">
<div id="gameTitle">🍉 FRUIT NINJA <span class="byline">By MD RAZ</span></div>
<div id="countdownBox">60</div>
</div>

<div id="gameover">
<div id="dangerSign">⚠️ DANGER ⚠️</div>
<div id="gameoverText">GAME OVER</div>
<div id="finalScoreLabel">Total Score</div>
<div id="finalScore">0</div>
<button id="playAgainBtn">PLAY AGAIN</button>
</div>

<div id="hud">
<div id="stats">
<div>Score: <span id="score">0</span></div>
<div>Combo: <span id="combo">0</span></div>
</div>
<div id="controls">
<div id="status">Connecting...</div>
<button id="calibrate">CALIBRATE</button>
<button id="reset">RESET</button>
</div>
</div>

<div id="debug">GX: 0.0 | GY: 0.0 | GZ: 0.0 | AX: 0.0 | AY: 0.0 | AZ: 0.0</div>
<div id="message">Move your hand to control the blade</div>

<!-- COUNTDOWN OVERLAY -->
<div id="countdownOverlay">
<div id="countdownNumber">3</div>
<div id="countdownSub">CALIBRATING...</div>
</div>

<canvas id="game"></canvas>

<script>

// =====================================================
// CANVAS
// =====================================================

const canvas = document.getElementById("game");
const ctx = canvas.getContext("2d");

function resize(){
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;
}
resize();
window.addEventListener("resize", resize);

// =====================================================
// COUNTDOWN SYSTEM
// =====================================================

const countdownOverlay = document.getElementById("countdownOverlay");
const countdownNumber = document.getElementById("countdownNumber");
const countdownSub = document.getElementById("countdownSub");

let countdownActive = true;
let gameCanStart = false;

function startCountdown(callback){
countdownActive = true;
countdownOverlay.classList.remove("hidden");
countdownNumber.textContent = "3";
countdownSub.textContent = "CALIBRATING...";

let count = 3;

// Send calibration command to ESP32
if(socket && socket.readyState === WebSocket.OPEN){
socket.send("CALIBRATE");
}

const interval = setInterval(() => {
count--;
if(count > 0){
countdownNumber.textContent = count;
countdownSub.textContent = "CALIBRATING...";
} else if(count === 0){
countdownNumber.textContent = "START!";
countdownSub.textContent = "GO!";
} else {
clearInterval(interval);
countdownOverlay.classList.add("hidden");
countdownActive = false;
gameCanStart = true;
if(callback) callback();
}
}, 1000);
}

// =====================================================
// GAME STATE
// =====================================================

let score = 0;
let combo = 0;
let fruits = [];
let particles = [];
let lastSpawn = 0;
let gameStartTime = 0;
let gameOver = false;
let gameStarted = false;

let cursor = {
x: canvas.width / 2,
y: canvas.height / 2
};

// =====================================================
// BLADE TRAIL - INCREASED LENGTH
// =====================================================
// TRAIL_MAX_LENGTH increased from 260 to 450 for longer blade trail
// TRAIL_MIN_SEGMENT increased from 2 to 3 for smoother trail

let cursorTrail = [{ x: cursor.x, y: cursor.y }];
const TRAIL_MAX_LENGTH = 450;       // Increased for longer blade trail
const TRAIL_MIN_SEGMENT = 3;        // Slightly increased for smoother trail
const HIT_RADIUS_PADDING = 26;
const HEADER_HEIGHT = 126;
const GAME_DURATION_MS = 60000;

// =====================================================
// FRUIT TYPES
// =====================================================

const fruitTypes = ["🍎","🍊","🍉","🍌","🍐","🍇","🥝","🍓","🍍","🥭","🍑","🍋","🍈","🥥","🍒"];
const birdTypes = ["🐦","🐤","🦜","🐧","🦉"];
const BIRD_CHANCE = 0.22;

// =====================================================
// SPAWN
// =====================================================

function spawnEntity(){
const x = 80 + Math.random() * (canvas.width - 160);
const isBird = Math.random() < BIRD_CHANCE;
const emojiList = isBird ? birdTypes : fruitTypes;

fruits.push({
x: x,
y: canvas.height + 70,
vx: (Math.random() - .5) * (isBird ? 7 : 5),
vy: -(12 + Math.random() * 5),
radius: isBird ? 34 : 42,
emoji: emojiList[Math.floor(Math.random() * emojiList.length)],
rotation: Math.random() * 6.28,
rotationSpeed: (Math.random() - .5) * .12,
isBird: isBird,
sliced: false
});
}

// =====================================================
// DRAW FUNCTIONS
// =====================================================

function drawFruit(f){
ctx.save();
ctx.translate(f.x, f.y);
ctx.rotate(f.rotation);
if(f.isBird){ ctx.shadowBlur = 20; ctx.shadowColor = "#ff1744"; }
ctx.font = `${f.radius*2}px Arial`;
ctx.textAlign = "center";
ctx.textBaseline = "middle";
ctx.fillText(f.emoji, 0, 0);
ctx.restore();
}

function explode(x,y,color){
for(let i=0; i<22; i++){
particles.push({
x:x, y:y,
vx: (Math.random()-.5)*10,
vy: (Math.random()-.5)*10,
life:1,
size: 2+Math.random()*6,
color: color || "white"
});
}
}

function updateParticles(){
for(const p of particles){
p.x += p.vx;
p.y += p.vy;
p.vy += .25;
p.life -= .035;
}
particles = particles.filter(p=>p.life>0);
}

function drawParticles(){
for(const p of particles){
ctx.globalAlpha = p.life;
ctx.beginPath();
ctx.arc(p.x, p.y, p.size, 0, Math.PI*2);
ctx.fillStyle = p.color || "white";
ctx.fill();
}
ctx.globalAlpha=1;
}

// =====================================================
// COLLISION
// =====================================================

function lineCircle(x1,y1,x2,y2,cx,cy,r){
const dx = x2-x1;
const dy = y2-y1;
const len = dx*dx+dy*dy;
if(len===0) return false;
let t = ((cx-x1)*dx + (cy-y1)*dy)/len;
t = Math.max(0, Math.min(1, t));
const px = x1+t*dx;
const py = y1+t*dy;
const ddx = cx-px;
const ddy = cy-py;
return (ddx*ddx+ddy*ddy < r*r);
}

function trackCursorAndSlice(){
const last = cursorTrail[cursorTrail.length - 1];
const dx = cursor.x - last.x;
const dy = cursor.y - last.y;
const dist = Math.sqrt(dx*dx + dy*dy);

if(dist < TRAIL_MIN_SEGMENT){
sliceAlongSegment(last.x, last.y, cursor.x, cursor.y);
return;
}

cursorTrail.push({x: cursor.x, y: cursor.y});

let totalLen = 0;
for(let i = cursorTrail.length - 1; i > 0; i--){
const a = cursorTrail[i];
const b = cursorTrail[i-1];
const segLen = Math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
totalLen += segLen;
if(totalLen > TRAIL_MAX_LENGTH){
cursorTrail = cursorTrail.slice(i);
break;
}
}
sliceAlongSegment(last.x, last.y, cursor.x, cursor.y);
}

function sliceAlongSegment(x1,y1,x2,y2){
let hits = 0;
for(const f of fruits){
if(f.sliced) continue;
const hitRadius = f.radius + HIT_RADIUS_PADDING;
if(lineCircle(x1,y1,x2,y2,f.x,f.y,hitRadius)){
f.sliced = true;
hits++;
if(f.isBird){
score = Math.max(0, score - 15);
combo = 0;
explode(f.x, f.y, "#ff1744");
} else {
score += 10 + combo * 2;
combo++;
explode(f.x, f.y, "white");
}
}
}
if(hits > 0) updateHUD();
}

// =====================================================
// DRAW TRAIL
// =====================================================

function drawTrail(){
if(cursorTrail.length === 0) return;
ctx.save();
ctx.lineCap = "round";
ctx.lineJoin = "round";
ctx.shadowBlur = 22;
ctx.shadowColor = "#00e5ff";

const n = cursorTrail.length;
for(let i=1; i<n; i++){
const a = cursorTrail[i-1];
const b = cursorTrail[i];
const f = i / (n - 1);
ctx.beginPath();
ctx.moveTo(a.x, a.y);
ctx.lineTo(b.x, b.y);
ctx.lineWidth = 2 + f * 9;
ctx.globalAlpha = 0.15 + f * 0.75;
ctx.strokeStyle = "white";
ctx.stroke();
}
ctx.restore();

const head = cursorTrail[n-1];
ctx.save();
ctx.beginPath();
ctx.arc(head.x, head.y, 9, 0, Math.PI*2);
ctx.fillStyle = "white";
ctx.shadowBlur = 30;
ctx.shadowColor = "#00e5ff";
ctx.fill();
ctx.restore();
}

// =====================================================
// GAME UPDATE
// =====================================================

function update(){
const now = performance.now();

if(!gameOver && gameStarted){
const remaining = Math.max(0, GAME_DURATION_MS - (now - gameStartTime));
updateCountdown(remaining);

if(remaining <= 0){
triggerGameOver();
} else {
if(now-lastSpawn > 720){
spawnEntity();
lastSpawn = now;
}
for(const f of fruits){
f.x += f.vx;
f.y += f.vy;
f.vy += .30;
f.rotation += f.rotationSpeed;
}
fruits = fruits.filter(f => f.y < canvas.height+150 && !f.sliced);
}
}
updateParticles();
}

function updateCountdown(remainingMs){
const secs = Math.ceil(remainingMs/1000);
const el = document.getElementById("countdownBox");
el.textContent = secs;
if(remainingMs <= 3000){
el.classList.add("warn");
} else {
el.classList.remove("warn");
}
}

function triggerGameOver(){
if(gameOver) return;
gameOver = true;
fruits = [];
document.getElementById("finalScore").textContent = score;
document.getElementById("gameover").style.display = "flex";
}

function startNewRound(){
gameOver = false;
score = 0;
combo = 0;
fruits = [];
particles = [];
lastSpawn = 0;
gameStarted = false;

document.getElementById("gameover").style.display = "none";
const cd = document.getElementById("countdownBox");
cd.textContent = 60;
cd.classList.remove("warn");

cursor.x = canvas.width/2;
cursor.y = canvas.height/2;
cursorTrail = [{ x: cursor.x, y: cursor.y }];
updateHUD();

// Start countdown
startCountdown(() => {
gameStarted = true;
gameStartTime = performance.now();
gameOver = false;
});
}

// =====================================================
// BACKGROUND
// =====================================================

function drawBackground(){
const g = ctx.createLinearGradient(0, HEADER_HEIGHT, 0, canvas.height);
g.addColorStop(0, "#14213d");
g.addColorStop(.5, "#07111f");
g.addColorStop(1, "#010309");
ctx.fillStyle = g;
ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawTimeWarning(){
if(gameOver || !gameStarted) return;
const remaining = Math.max(0, GAME_DURATION_MS - (performance.now() - gameStartTime));
if(remaining > 3000) return;
const t = 1 - (remaining/3000);
ctx.save();
ctx.fillStyle = `rgba(0,0,0,${0.12 + t*0.45})`;
ctx.fillRect(0,0,canvas.width,canvas.height);
const cx = canvas.width/2;
const cy = canvas.height/2;
const outerR = Math.max(canvas.width,canvas.height) * (0.85 - t*0.25);
const grad = ctx.createRadialGradient(cx,cy, outerR*0.55, cx,cy, outerR);
grad.addColorStop(0,"rgba(255,23,68,0)");
grad.addColorStop(1,`rgba(255,23,68,${0.35+t*0.5})`);
ctx.fillStyle = grad;
ctx.fillRect(0,0,canvas.width,canvas.height);
ctx.restore();
}

// =====================================================
// HUD
// =====================================================

function updateHUD(){
document.getElementById("score").textContent = score;
document.getElementById("combo").textContent = combo;
}

// =====================================================
// LOOP
// =====================================================

function loop(){
drawBackground();
update();
for(const f of fruits) drawFruit(f);
drawParticles();
drawTrail();
drawTimeWarning();
requestAnimationFrame(loop);
}
loop();

// =====================================================
// WEBSOCKET
// =====================================================

let socket = null;

function connect(){
const protocol = location.protocol === "https:" ? "wss://" : "ws://";
socket = new WebSocket(protocol + location.hostname + ":81/");

socket.onopen = function(){
document.getElementById("status").textContent = "🟢 Connected";
document.getElementById("status").style.color = "#00e676";
document.getElementById("message").textContent = "";
// Start countdown when connected
startNewRound();
};

socket.onclose = function(){
document.getElementById("status").textContent = "🔴 Disconnected";
document.getElementById("status").style.color = "#ff5252";
setTimeout(connect, 2000);
};

socket.onmessage = function(event){
let d;
try { d = JSON.parse(event.data); }
catch(e) { return; }

if(d.type === "motion"){
document.getElementById("debug").textContent =
"GX: "+d.gx.toFixed(1)+" | GY: "+d.gy.toFixed(1)+" | GZ: "+d.gz.toFixed(1)+
" | AX: "+d.ax.toFixed(2)+" | AY: "+d.ay.toFixed(2)+" | AZ: "+d.az.toFixed(2);

// =====================================================
// MOUSE-LIKE CONTROLLER WITH TILT SPEED
// =====================================================
// 
// HOW IT WORKS:
// - Gyro Y (gy) = LEFT/RIGHT movement
// - Gyro X (gx) = UP/DOWN movement
// - Accelerometer tilt = Speed boost
//
// FIXED DIRECTIONS:
// - Move right = gy positive → cursor moves right ✓
// - Move left = gy negative → cursor moves left ✓
// - Move forward (tilt up) = gx negative → cursor moves up ✓
// - Move backward (tilt down) = gx positive → cursor moves down ✓
//
// =====================================================
// TO REVERSE DIRECTIONS:
// =====================================================
// Left/Right: change "+gy" to "-gy"
// Up/Down: change "-gx" to "+gx"
// =====================================================

const GYRO_DEADZONE = 0.8;          // Increased for stability
const GYRO_BASE_SPEED = 0.9;        // Reduced for slower movement
const MAX_SPEED_MULTIPLIER = 2.4;   // Reduced max speed
const TILT_BOOST_FACTOR = 1.4;      // Reduced tilt effect

// Apply deadzone
let gx = Math.abs(d.gx) < GYRO_DEADZONE ? 0 : d.gx;
let gy = Math.abs(d.gy) < GYRO_DEADZONE ? 0 : d.gy;

// Extra filtering for micro-jitter
if(Math.abs(gx) < 0.15) gx = 0;
if(Math.abs(gy) < 0.15) gy = 0;

// Tilt magnitude from accelerometer
const tiltMagnitude = Math.sqrt(d.ax*d.ax + d.ay*d.ay);
let speedMultiplier = 1.0 + (tiltMagnitude * TILT_BOOST_FACTOR);
speedMultiplier = Math.min(speedMultiplier, MAX_SPEED_MULTIPLIER);

// Smooth curve for natural feel
function curveInput(v){
const sign = v > 0 ? 1 : -1;
const absV = Math.abs(v);
return sign * (absV * absV * 0.1 + absV * 0.9);
}

const speed = GYRO_BASE_SPEED * speedMultiplier;

// =====================================================
// DIRECTION CONTROLS - FIXED!
// =====================================================
// RIGHT = positive gy → cursor.x increases ✓
// LEFT = negative gy → cursor.x decreases ✓
// UP = negative gx → cursor.y decreases ✓
// DOWN = positive gx → cursor.y increases ✓
// =====================================================

cursor.x += -curveInput(gy) * speed * 0.8;    // Horizontal
cursor.y += curveInput(gx) * speed * 0.8;   // Vertical

// Clamp cursor
cursor.x = Math.max(0, Math.min(canvas.width, cursor.x));
cursor.y = Math.max(HEADER_HEIGHT, Math.min(canvas.height, cursor.y));

// Only track cursor if game has started
if(gameStarted && !gameOver){
trackCursorAndSlice();
}

document.getElementById("debug").textContent += 
" | Speed: "+speedMultiplier.toFixed(1)+"x";
}

if(d.type === "calibration"){
document.getElementById("status").textContent = d.message;
if(d.message === "Calibrated!"){
document.getElementById("status").textContent = "✅ Calibrated";
document.getElementById("status").style.color = "#00e676";
}
}
};
}

connect();

// =====================================================
// MOUSE CONTROL (for testing)
// =====================================================

canvas.addEventListener("mousemove", function(e){
if(!gameStarted || gameOver) return;
const rect = canvas.getBoundingClientRect();
cursor.x = (e.clientX - rect.left) * (canvas.width / rect.width);
cursor.y = (e.clientY - rect.top) * (canvas.height / rect.height);
cursor.x = Math.max(0, Math.min(canvas.width, cursor.x));
cursor.y = Math.max(HEADER_HEIGHT, Math.min(canvas.height, cursor.y));
trackCursorAndSlice();
});

// =====================================================
// BUTTONS
// =====================================================

document.getElementById("calibrate").onclick = function(){
if(socket && socket.readyState === WebSocket.OPEN){
socket.send("CALIBRATE");
}
};

document.getElementById("reset").onclick = function(){
particles = [];
startNewRound();
};

document.getElementById("playAgainBtn").onclick = function(){
particles = [];
startNewRound();
};

</script>
</body>
</html>

)HTML";

// =====================================================
// ROOT
// =====================================================

void handleRoot()
{
  server.send_P(200, "text/html", GAME_HTML);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-S3 MPU6500 FRUIT NINJA");
  Serial.println("================================");

  // -----------------------------
  // I2C
  // -----------------------------

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  // -----------------------------
  // MPU
  // -----------------------------

  if (!initMPU())
  {
    Serial.println("MPU initialization failed!");
  }

  // -----------------------------
  // WIFI (Access Point / Hotspot)
  // -----------------------------

  WiFi.mode(WIFI_AP);

  // Explicitly force the AP address to 192.168.4.1
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);

  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  delay(200);

  Serial.println();
  Serial.println("WiFi Hotspot started!");
  Serial.print("Network name (SSID): ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 IP (fixed): ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Connect your phone/PC to this WiFi, then open the IP above in a browser.");

  // -----------------------------
  // HTTP
  // -----------------------------

  server.on("/", handleRoot);
  server.begin();

  // -----------------------------
  // WebSocket
  // -----------------------------

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println();
  Serial.println("HTTP Server: Port 80");
  Serial.println("WebSocket: Port 81");
  Serial.println("================================");
  Serial.println("SYSTEM READY");
  Serial.println("================================");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  server.handleClient();
  webSocket.loop();

  static unsigned long lastRead=0;
  unsigned long now = millis();

  if(now-lastRead >= 20)
  {
    lastRead=now;

    if(readMPU())
    {
      float cgx = gx-gyroOffsetX;
      float cgy = gy-gyroOffsetY;
      float cgz = gz-gyroOffsetZ;
      float cax = ax-accelOffsetX;
      float cay = ay-accelOffsetY;
      float caz = az-accelOffsetZ;

      fgx = FILTER_ALPHA*cgx + (1.0-FILTER_ALPHA)*fgx;
      fgy = FILTER_ALPHA*cgy + (1.0-FILTER_ALPHA)*fgy;
      fgz = FILTER_ALPHA*cgz + (1.0-FILTER_ALPHA)*fgz;
      fax = FILTER_ALPHA*cax + (1.0-FILTER_ALPHA)*fax;
      fay = FILTER_ALPHA*cay + (1.0-FILTER_ALPHA)*fay;
      faz = FILTER_ALPHA*caz + (1.0-FILTER_ALPHA)*faz;

      char json[220];
      snprintf(json, sizeof(json),
        "{\"type\":\"motion\",\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f}",
        fgx, fgy, fgz, fax, fay, faz
      );
      webSocket.broadcastTXT(json);

      // Gesture detection
      float gyroSpeed = sqrt(fgx*fgx + fgy*fgy);
      float accelMagnitude = sqrt(fax*fax + fay*fay + faz*faz);
      static float prevAccelMag = 0;
      float accelDelta = accelMagnitude - prevAccelMag;
      prevAccelMag = accelMagnitude;
      
      float accelBoost = 0;
      if (accelDelta > 0.1f) {
        accelBoost = accelDelta * 15.0f;
      }
      float totalSpeed = gyroSpeed + accelBoost;

      if(!slashActive && totalSpeed > SLASH_START && now-lastSlash > SLASH_COOLDOWN)
      {
        slashActive=true;
        slashStart=now;
        slashDX=0;
        slashDY=0;
        slashPeak=totalSpeed;
      }

      if(slashActive)
      {
        slashDX += (fgy + fax * 2.0f) * 0.020f;
        slashDY += (fgx + fay * 2.0f) * 0.020f;

        if(totalSpeed > slashPeak) slashPeak = totalSpeed;

        bool finished = totalSpeed < SLASH_END;
        bool timeout = now-slashStart > MAX_SLASH_TIME;

        if(finished || timeout)
        {
          float length = sqrt(slashDX*slashDX + slashDY*slashDY);
          if(length > 15)
          {
            float power = slashPeak/600.0f;
            if(power>1) power=1;
            if(power<0.15) power=0.15;

            char slashJSON[180];
            snprintf(slashJSON, sizeof(slashJSON),
              "{\"type\":\"slash\",\"dx\":%.2f,\"dy\":%.2f,\"power\":%.2f}",
              slashDX, slashDY, power
            );
            webSocket.broadcastTXT(slashJSON);
            lastSlash = now;
          }
          slashActive=false;
          slashDX=0;
          slashDY=0;
          slashPeak=0;
        }
      }
    }
  }
}