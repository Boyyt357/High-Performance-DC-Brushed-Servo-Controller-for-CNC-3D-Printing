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

// --- PID & Control Variables ---
float Kp, Ki, Kd, tolerance, maxDeg, gearRatio;
int microsteps;
float targetDeg = 0.0, currentDeg = 0.0, currentError = 0.0;
float integral = 0, lastError = 0;
bool servoEnabled = true;

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

void resetPID() {
  integral = 0;
  lastError = 0;
}

void driveMotor(int output) {
  float error = abs(targetDeg - currentDeg);

  // DEADZONE: Stop motor if within tolerance to prevent jitter/humming
  if (!servoEnabled || error < tolerance) {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, 0);
    return;
  }

  // Minimum starting power (constrain between 45 and 255)
  int speed = constrain(abs(output), 45, 255); 
  
  if (output > 0) {
    analogWrite(motorIN1, speed);
    analogWrite(motorIN2, 0);
  } else {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, speed);
  }
}

void runPID() {
  currentError = targetDeg - currentDeg;
  
  // Anti-windup for Integral
  integral += currentError;
  integral = constrain(integral, -100, 100); 
  
  float derivative = currentError - lastError;
  int output = (Kp * currentError) + (Ki * integral) + (Kd * derivative);
  
  driveMotor(output);
  lastError = currentError;
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Klipper Servo Panel</title>
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
    <h2>Klipper Closed-Loop Servo</h2>
    <div class="stats">
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Target&deg;</div><div class="stat-val" id="tgt">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Current&deg;</div><div class="stat-val" id="pos" style="color:#2ecc71;">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Error&deg;</div><div class="stat-val err-val" id="err">0.0</div></div>
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
    <h4>PID Tuning</h4>
    <div class="row"> P:<input type="text" id="p"> I:<input type="text" id="i"> D:<input type="text" id="d"> </div>
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
    const p = document.getElementById('p').value, i = document.getElementById('i').value, d = document.getElementById('d').value;
    const t = document.getElementById('t').value, max = document.getElementById('mDeg').value, r = document.getElementById('ratio').value;
    const ms = document.getElementById('msteps').value;
    fetch(`/save?p=${p}&i=${i}&d=${d}&t=${t}&max=${max}&r=${r}&ms=${ms}`).then(() => alert("Data Saved Permanently!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('p').value = data.p; document.getElementById('i').value = data.i;
    document.getElementById('d').value = data.d; document.getElementById('t').value = data.t;
    document.getElementById('mDeg').value = data.max; document.getElementById('ratio').value = data.r;
    document.getElementById('msteps').value = data.ms;
  });
  setInterval(() => { 
    fetch('/getlive').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt; 
      document.getElementById('err').innerText = data.err; 
    }); 
  }, 200);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
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
  Kp = prefs.getFloat("kp", 2.0);
  Ki = prefs.getFloat("ki", 0.0);
  Kd = prefs.getFloat("kd", 0.1);
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
  
  // Back-calculate target_steps so the internal step counter matches current reality
  target_steps = (targetDeg / 360.0) * (200.0 * microsteps);

  // Attach the STEP interrupt
  attachInterrupt(digitalPinToInterrupt(stepPin), handleStep, RISING);

  WiFi.softAP("SmartServo_Pro", "");

  // API Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    Kp = f->getParam("p")->value().toFloat();
    Ki = f->getParam("i")->value().toFloat();
    Kd = f->getParam("d")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    microsteps = f->getParam("ms")->value().toInt();

    resetPID();

    prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
    prefs.putFloat("tol", tolerance); prefs.putFloat("maxDeg", maxDeg); 
    prefs.putFloat("ratio", gearRatio); prefs.putInt("ms", microsteps);
    f->send(200);
  });

  // Web Move (temporarily overwrites Klipper's step command for testing)
  server.on("/move", [](AsyncWebServerRequest *f){ 
    targetDeg = f->getParam("val")->value().toFloat(); 
    target_steps = (targetDeg / 360.0) * (200.0 * microsteps); // Sync the step counter
    f->send(200); 
  });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"p\":"+String(Kp)+",\"i\":"+String(Ki)+",\"d\":"+String(Kd)+",\"t\":"+String(tolerance)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+",\"ms\":"+String(microsteps)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/getlive", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\""+String(currentDeg, 2)+"\",\"tgt\":\""+String(targetDeg, 2)+"\",\"err\":\""+String(abs(currentError), 2)+"\"}";
    f->send(200, "application/json", json); 
  });
  
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0;
    target_steps = 0; // Reset step counter on home
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  updateEncoder();

  // Convert incoming Klipper steps into Target Degrees
  // Formula: (Steps / (200 full steps * Microsteps)) * 360 degrees
  targetDeg = (float(target_steps) / (200.0 * float(microsteps))) * 360.0;

  runPID();
  delay(1); 
}