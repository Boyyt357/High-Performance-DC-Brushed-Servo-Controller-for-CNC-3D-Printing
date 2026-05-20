#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// --- Pins (ESP32-S3) ---
const int motorIN1 = 1; 
const int motorIN2 = 2; 
const int motorENA = 18; 

// --- STEP / DIR Inputs from RP2350 ---
const int stepPin = 4; // Connect to RP2350 GPIO3
const int dirPin = 5;  // Connect to RP2350 GPIO2

// --- ADRC & Control Variables ---
float wc, wo, b0; // Bandwidth_c, Bandwidth_o, System Gain
float tolerance, maxDeg, gearRatio;
int microsteps;
float targetDeg = 0.0, currentDeg = 0.0, currentError = 0.0;
bool servoEnabled = true;

// ESO (Extended State Observer) States
float z1 = 0.0; // Estimated Position
float z2 = 0.0; // Estimated Velocity
float z3 = 0.0; // Estimated Disturbance (Friction/Weight)
float lastOutput = 0.0;
unsigned long lastTime = 0;

// --- Step Tracking ---
volatile long target_steps = 0;

// --- Encoder Variables ---
int lastRaw = 0;
long totalRaw = 0;
long homeOffset = 0;

Preferences prefs; 
AsyncWebServer server(80);

// --- Fast Interrupt for STEP Pin ---
void IRAM_ATTR handleStep() {
  // Read DIR pin to determine direction
  if (digitalRead(dirPin) == HIGH) {
    target_steps++;
  } else {
    target_steps--;
  }
}

void updateEncoder() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // AS5600 Angle Register
  if (Wire.endTransmission() != 0) return; // I2C Error check

  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    int diff = raw - lastRaw;
    
    // Handle wrap-around for multi-turn
    if (diff > 2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    
    totalRaw += diff;
    lastRaw = raw;

    // Calculate current degrees
    currentDeg = ((totalRaw - homeOffset) * (360.0 / 4096.0)) * gearRatio;
  }
}

void driveMotor(float output) {
  float error = abs(targetDeg - currentDeg);

  // DEADZONE: Stop motor if within tolerance to prevent jitter/humming
  if (!servoEnabled || error < tolerance) {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, 0);
    lastOutput = 0.0; // Tell Observer the motor is OFF
    return;
  }

  // Minimum starting power (constrain between 45 and 255)
  int speed = constrain(abs((int)output), 45, 255); 
  
  if (output > 0) {
    analogWrite(motorIN1, speed);
    analogWrite(motorIN2, 0);
    lastOutput = speed; // Update Observer with actual effort
  } else {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, speed);
    lastOutput = -speed; // Update Observer with actual effort
  }
}

void runADRC() {
  currentError = targetDeg - currentDeg;
  
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.05f) { lastTime = now; return; } // Prevent dt explosion
  lastTime = now;

  // 1. Observer Gains
  float beta1 = 3.0f * wo;
  float beta2 = 3.0f * (wo * wo);
  float beta3 = (wo * wo * wo);

  // 2. Extended State Observer (ESO) Update
  float error_eso = z1 - currentDeg;
  
  z1 += dt * (z2 - beta1 * error_eso);
  z2 += dt * (z3 + (b0 * lastOutput) - beta2 * error_eso);
  z3 += dt * (-beta3 * error_eso); 

  // 3. Control Law (Virtual PD)
  float kp = wc * wc;
  float kd = 2.0f * wc;
  
  float u0 = kp * (targetDeg - z1) - kd * z2;

  // 4. Disturbance Rejection
  float u = (u0 - z3) / b0;

  // Constrain to Arduino's 8-bit analogWrite range (-255 to 255)
  float safeOutput = constrain(u, -255.0f, 255.0f);
  
  driveMotor(safeOutput);
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Klipper ADRC Servo</title>
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 420px; border: 1px solid #444; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; }
  .row input { width: 60px; }
  .stats { display: flex; justify-content: space-between; background: #111; padding: 10px; border-radius: 8px; margin-bottom: 15px; }
  .stat-box { width: 30%; }
  .stat-val { font-size: 1.5em; font-weight: bold; }
  .err-val { color: #e74c3c; }
</style></head>
<body>
  <div class="card">
    <h2>Klipper Closed-Loop ADRC</h2>
    <div class="stats">
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Target&deg;</div><div class="stat-val" id="tgt">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Current&deg;</div><div class="stat-val" id="pos" style="color:#2ecc71;">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Dist (z3)</div><div class="stat-val err-val" id="dist">0.0</div></div>
    </div>
    <hr>
    <h4>Mechanics & Klipper Settings</h4>
    <div class="row"> 
      Gear Ratio: <input type="text" id="ratio" style="width:50px;"> 
      Microsteps: 
      <select id="msteps" style="width:80px; margin:0;">
        <option value="1">1</option><option value="2">1/2</option><option value="4">1/4</option>
        <option value="8">1/8</option><option value="16">1/16</option><option value="32">1/32</option>
      </select>
    </div>
    <hr>
    <h4>ADRC Tuning</h4>
    <div class="row"> &#969;c (Stiffness):<input type="text" id="wc"> &#969;o (Observer):<input type="text" id="wo"> b0 (Motor Power):<input type="text" id="b0"> </div>
    <div class="row"> Tolerance&deg;:<input type="text" id="t"> Max Limits&deg;:<input type="text" id="mDeg"> </div>
    <button class="btn save" onclick="saveSettings()">APPLY & SAVE</button>
    <hr>
    <button class="btn on" onclick="fetch('/sethome')">ZERO HOME (Sync)</button>
    <input type="number" id="moveVal" placeholder="Test Target Degrees" style="width: 40%;">
    <button class="btn on" style="width:45%; background:#9b59b6" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function saveSettings() {
    const wc = document.getElementById('wc').value, wo = document.getElementById('wo').value, b0 = document.getElementById('b0').value;
    const t = document.getElementById('t').value, max = document.getElementById('mDeg').value, r = document.getElementById('ratio').value;
    const ms = document.getElementById('msteps').value;
    fetch(`/save?wc=${wc}&wo=${wo}&b0=${b0}&t=${t}&max=${max}&r=${r}&ms=${ms}`).then(() => alert("Data Saved Permanently!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('wc').value = data.wc; document.getElementById('wo').value = data.wo;
    document.getElementById('b0').value = data.b0; document.getElementById('t').value = data.t;
    document.getElementById('mDeg').value = data.max; document.getElementById('ratio').value = data.r;
    document.getElementById('msteps').value = data.ms;
  });
  setInterval(() => { 
    fetch('/getlive').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt; 
      document.getElementById('dist').innerText = data.dist; 
    }); 
  }, 200);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  Wire.setClock(1000000); // 1MHz I2C for faster encoder reading
  
  pinMode(motorIN1, OUTPUT); 
  pinMode(motorIN2, OUTPUT);
  pinMode(motorENA, OUTPUT); 
  digitalWrite(motorENA, HIGH);
  
  pinMode(stepPin, INPUT_PULLDOWN);
  pinMode(dirPin, INPUT_PULLDOWN);

  analogWriteFrequency(motorIN1, 20000); 
  analogWriteFrequency(motorIN2, 20000);

  // Load Saved Data
  prefs.begin("servo-data", false);
  homeOffset = prefs.getLong("offset", 0);
  wc = prefs.getFloat("wc", 30.0);   // Defaults for ADRC
  wo = prefs.getFloat("wo", 100.0);
  b0 = prefs.getFloat("b0", 250.0);
  tolerance = prefs.getFloat("tol", 0.5);
  maxDeg = prefs.getFloat("maxDeg", 180.0);
  gearRatio = prefs.getFloat("ratio", 1.0);
  microsteps = prefs.getInt("ms", 16);

  // INITIAL ENCODER READ
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    lastRaw = (Wire.read() << 8) | Wire.read();
    totalRaw = lastRaw; 
  }

  // Calculate current pos and SYNC target so motor holds position on boot
  updateEncoder();
  targetDeg = currentDeg; 
  
  // Initialize the ADRC observer states so it doesn't jerk on startup
  z1 = currentDeg;
  z2 = 0.0;
  z3 = 0.0;
  lastTime = micros();
  
  // Back-calculate target_steps so the internal step counter matches current reality
  target_steps = (targetDeg / 360.0) * (200.0 * microsteps);

  // Attach the STEP interrupt
  attachInterrupt(digitalPinToInterrupt(stepPin), handleStep, RISING);

  WiFi.softAP("SmartServo_Pro", "");

  // API Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    wc = f->getParam("wc")->value().toFloat();
    wo = f->getParam("wo")->value().toFloat();
    b0 = f->getParam("b0")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    microsteps = f->getParam("ms")->value().toInt();

    prefs.putFloat("wc", wc); prefs.putFloat("wo", wo); prefs.putFloat("b0", b0);
    prefs.putFloat("tol", tolerance); prefs.putFloat("maxDeg", maxDeg); 
    prefs.putFloat("ratio", gearRatio); prefs.putInt("ms", microsteps);
    f->send(200);
  });

  server.on("/move", [](AsyncWebServerRequest *f){ 
    targetDeg = f->getParam("val")->value().toFloat(); 
    target_steps = (targetDeg / 360.0) * (200.0 * microsteps);
    f->send(200); 
  });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"wc\":"+String(wc)+",\"wo\":"+String(wo)+",\"b0\":"+String(b0)+",\"t\":"+String(tolerance)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+",\"ms\":"+String(microsteps)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/getlive", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\""+String(currentDeg, 2)+"\",\"tgt\":\""+String(targetDeg, 2)+"\",\"dist\":\""+String(z3, 1)+"\"}";
    f->send(200, "application/json", json); 
  });
  
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0;
    target_steps = 0; 
    z1 = 0; // Reset observer position
    z2 = 0; z3 = 0;
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  updateEncoder();

  // Convert incoming Klipper steps into Target Degrees
  targetDeg = (float(target_steps) / (200.0 * float(microsteps))) * 360.0;

  runADRC();
  delay(1); 
}