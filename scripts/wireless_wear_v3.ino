#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <DynamixelShield.h>

// ---------- WiFi ----------
char ssid[] = "Garage";
char pass[] = "Robotz0nly";

WiFiUDP Udp;

// Ports
const unsigned int WEARABLE_LOCAL_PORT = 4210; // wearable listens here for F: packets
const unsigned int POS_SEND_PORT       = 4211; // EE listens here for follower position
IPAddress eeIP(192, 168, 0, 32);               // EE's IP

// ---------- DYNAMIXEL (XL330-M288-T as master) ----------
DynamixelShield dxl;
const float PROTOCOL_VERSION = 2.0;
const uint8_t ID_MASTER = 2;

// ---------- Force params ----------
int   FSR_AVG          = 0;    // from EE (filtered there)
bool  forceActive      = false;
unsigned long lastForceUpdate = 0;

// FIX: match EE's separate ON/OFF thresholds to prevent flickering at boundary
const int   FORCE_ON_THRESHOLD  = 20;   // FSR level where haptic kicks in
const int   FORCE_OFF_THRESHOLD = 10;   // FSR level where haptic releases
const float FORCE_GAIN          = 0.5;  // tune up/down for stronger/weaker feel
const float MAX_OFFSET_DEG      = 15.0; // limit virtual spring offset for XL330

// Wearable motion limits
const float MASTER_CLOSED_POS = 110.0;  // fully closed
const float MASTER_OPEN_POS   = 40.0;   // fully open

// EE output limits
const float EE_CLOSED_POS = 130.0;
const float EE_OPEN_POS   = 250.0;

// Timing
const unsigned long POS_SEND_INTERVAL_MS = 5;   // ~200 Hz position updates to EE
unsigned long lastPosSend = 0;

// Buffers
char incomingPacket[32];

// FIX: hapticActive tracks the actual torque state and is only toggled on
//      transitions — never written every loop iteration, which was flooding
//      the Dynamixel bus and causing the servo to stay in a bad state.
bool hapticActive = false;

// FIX: float-safe map — Arduino's built-in map() truncates to long integers,
//      causing quantization and wrong boundary values with float positions.
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.println("Wearable: Connecting to WiFi...");
    status = WiFi.begin(ssid, pass);
    delay(200);
  }

  Serial.print("Wearable connected. IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(WEARABLE_LOCAL_PORT);
  Serial.print("Wearable listening on UDP port ");
  Serial.println(WEARABLE_LOCAL_PORT);

  Serial.print("Wearable IP: "); Serial.println(WiFi.localIP()); // FIX: was "EE IP:"

  // ---- Dynamixel init for XL330 ----
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);

  // Reboot servo to clear any latched hardware error flags left by
  // previous out-of-range setGoalPosition commands.
  dxl.reboot(ID_MASTER);
  delay(300); // wait for XL330 to come back online after reboot

  // FIX: explicitly set Position Control Mode (OP_POSITION = 3).
  // reboot() restores RAM from EEPROM — if a previous sketch left Operating
  // Mode set to Velocity (1) or Current (0) in EEPROM, setGoalPosition is
  // silently ignored every time even though torqueOn() succeeds. The motor
  // energises (resists manual movement) but never drives to a goal position.
  // Operating Mode is an EEPROM register that can only be written while
  // Torque Enable = 0, which is the state after reboot, so this is safe here.
  dxl.setOperatingMode(ID_MASTER, OP_POSITION);
  // Should print 3. If it prints anything else, the write failed — check
  // that your DynamixelShield library is up to date and the servo ID is correct.

  dxl.torqueOff(ID_MASTER);

  Serial.println("Wearable Ready (XL330 + position + virtual spring).");
}

void loop() {
  unsigned long now = millis();

  // --- 0. Receive force packets from EE (F:<value>) ---
  int packetSize;
  while ((packetSize = Udp.parsePacket()) > 0) {
    int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len <= 0) continue;
    incomingPacket[len] = '\0';

    if (strncmp(incomingPacket, "F:", 2) == 0) {
      FSR_AVG = atoi(incomingPacket + 2);
      forceActive = true;
      lastForceUpdate = now;
    }
  }

  // Force timeout: if no F packets for 200 ms, treat as no force
  if (forceActive && (now - lastForceUpdate > 200)) {
    forceActive = false;
    FSR_AVG = 0;
  }

  // --- 1. Update haptic state with hysteresis; toggle torque only on transitions ---
  // FIX: previously torqueOn/torqueOff were called on EVERY loop iteration,
  //      generating 2-3 Dynamixel bus writes per 5 ms and causing the servo
  //      to see a torqueOff followed immediately by torqueOn+setGoalPosition,
  //      leaving it in an undetermined state. Now torque is only touched when
  //      the state actually changes.
  bool newHaptic = hapticActive;
  if (!hapticActive && forceActive && FSR_AVG >= FORCE_ON_THRESHOLD) {
    newHaptic = true;
  } else if (hapticActive && (!forceActive || FSR_AVG < FORCE_OFF_THRESHOLD)) {
    newHaptic = false;
  }
  if (newHaptic != hapticActive) {
    hapticActive = newHaptic;
    if (hapticActive) dxl.torqueOn(ID_MASTER);
    else              dxl.torqueOff(ID_MASTER);
  }

  // --- 2. Always read master position (XL330 angle) ---
  float masterPos = dxl.getPresentPosition(ID_MASTER, UNIT_DEGREE);

  // Debug (throttled to 20 Hz so it doesn't choke serial)
  static unsigned long lastDebugPrint = 0;
  if (now - lastDebugPrint >= 50) {
    lastDebugPrint = now;
    Serial.print("MasterPos=");
    Serial.print(masterPos, 1);
    Serial.print(" | FSR_AVG=");
    Serial.print(FSR_AVG);
    Serial.print(" | hapticActive=");
    Serial.println(hapticActive ? "YES" : "NO");
  }

  // --- 3. Compute follower goal and master behavior ---
  float followerGoal;

  if (!hapticActive) {
    // POSITION-ONLY MODE: torque off (handled above), follower tracks master
    float constrainedMasterPos = constrain(masterPos, MASTER_OPEN_POS, MASTER_CLOSED_POS);

    // Inverted mapping: wearable open(40)→EE open(250), closed(110)→EE closed(130)
    followerGoal = fmap(constrainedMasterPos,
                        MASTER_OPEN_POS, MASTER_CLOSED_POS,
                        EE_OPEN_POS,     EE_CLOSED_POS);

  } else {
    // FORCE FEEDBACK MODE: virtual spring resists further closing
    // offset pushes goalPosMaster toward OPEN (smaller values),
    // which is the direction opposite to closing (110).
    float offset = (FSR_AVG - FORCE_ON_THRESHOLD) * FORCE_GAIN;
    if (offset > MAX_OFFSET_DEG) offset = MAX_OFFSET_DEG;
    if (offset < 0)              offset = 0; // offset only ever positive

    float goalPosMaster        = masterPos - offset;
    // FIX: constrain BEFORE sending to servo — unconstrained out-of-range
    //      commands were triggering the XL330 hardware error flag, which
    //      auto-disables torque and ignores all subsequent torqueOn() calls
    //      until the servo is rebooted.
    float constrainedMasterPos = constrain(goalPosMaster, MASTER_OPEN_POS, MASTER_CLOSED_POS);

    // Map constrained position to EE space (same inversion as position mode)
    followerGoal = fmap(constrainedMasterPos,
                        MASTER_OPEN_POS, MASTER_CLOSED_POS,
                        EE_OPEN_POS,     EE_CLOSED_POS);

    // FIX: was goalPosMaster (unconstrained). Must use constrainedMasterPos.
    // Return value check: if this prints FAILED, the servo is still in the
    // wrong operating mode — verify the "Operating mode set. Read back: 3"
    // message printed during setup().
    bool cmdOk = dxl.setGoalPosition(ID_MASTER, constrainedMasterPos, UNIT_DEGREE);
    if (!cmdOk) Serial.println("setGoalPosition FAILED — check operating mode");
  }

  // Final safety clamp on what gets sent to EE
  followerGoal = constrain(followerGoal, EE_CLOSED_POS, EE_OPEN_POS);

  // --- 4. Send followerGoal to EE at steady rate ---
  if (now - lastPosSend >= POS_SEND_INTERVAL_MS) {
    lastPosSend = now;
    String msg = String(followerGoal, 1);
    Udp.beginPacket(eeIP, POS_SEND_PORT);
    Udp.write(msg.c_str());
    Udp.endPacket();
  }

  // no delay()
}
